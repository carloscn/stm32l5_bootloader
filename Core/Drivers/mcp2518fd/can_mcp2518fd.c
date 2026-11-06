/*
 * hal_can backend: external MCP2518FD on SPI, built on Microchip's drv_canfdspi.
 *
 * Board settings come from hal_config.h:
 *   HAL_CONFIG_CAN1_MCP2518FD_OSC_HZ      crystal on the module (40/20/10 MHz)
 *   HAL_CONFIG_CAN1_MCP2518FD_SPI_MAX_HZ  SPI clock cap (0: controller maximum)
 *
 * Controller setup: TX FIFO 8 x 64 B, RX FIFO 16 x 64 B, one acceptance
 * filter, ISO CRC + ECC on, no TEF / TXQ / RX time stamps. The INT0 (TX) and
 * INT1 (RX) pins are configured but not used: TX/RX are polled.
 *
 * Do not include any stm32l5xx_*.h here: drv_canfdspi_defines.h declares
 * GPIO_PIN_0 / GPIO_PIN_1 enum constants that clash with STM32 HAL macros.
 */
#include <string.h>
#include "hal_can_backend.h"
#include "hal_config.h"
#include "hal_err.h"
#include "hal_utils.h"
#include "drv_canfdspi_port.h"
#include "drv_canfdspi_api.h"
#include "drv_canfdspi_register.h"
#include "drv_canfdspi_defines.h"

#if   HAL_CONFIG_CAN1_MCP2518FD_OSC_HZ == 40000000u
#define MCP_SYSCLK      CAN_SYSCLK_40M
#elif HAL_CONFIG_CAN1_MCP2518FD_OSC_HZ == 20000000u
#define MCP_SYSCLK      CAN_SYSCLK_20M
#elif HAL_CONFIG_CAN1_MCP2518FD_OSC_HZ == 10000000u
#define MCP_SYSCLK      CAN_SYSCLK_10M
#else
#error "HAL_CONFIG_CAN1_MCP2518FD_OSC_HZ must be 40000000u, 20000000u or 10000000u"
#endif

/* MCP2518FD: f_SCK <= 0.85 * SYSCLK / 2, SYSCLK = oscillator (no PLL). */
#define MCP_SPI_LIMIT_HZ    (HAL_CONFIG_CAN1_MCP2518FD_OSC_HZ / 200u * 85u)

#define MCP_TX_FIFO         CAN_FIFO_CH1
#define MCP_RX_FIFO         CAN_FIFO_CH2
#define MCP_RX_FILTER       CAN_FILTER0

/* FifoSize field is "objects - 1". 8 TX + 16 RX objects of 8 + 64 bytes use
 * 1728 bytes of the 2 KB message RAM. */
#define MCP_TX_FIFO_DEPTH   (8u)
#define MCP_RX_FIFO_DEPTH   (16u)

#define MCP_OSC_READY_TIMEOUT_MS    (10u)
/* Leaving configuration mode waits for 11 recessive bits on the bus. */
#define MCP_MODE_SWITCH_TIMEOUT_MS  (50u)

static const CAN_BITTIME_SETUP mcp_bitrate_map[HAL_CAN_BITRATE_MAX] = {
    [HAL_CAN_BITRATE_125K_500K] = CAN_125K_500K,
    [HAL_CAN_BITRATE_250K_500K] = CAN_250K_500K,
    [HAL_CAN_BITRATE_250K_1M]   = CAN_250K_1M,
    [HAL_CAN_BITRATE_250K_2M]   = CAN_250K_2M,
    [HAL_CAN_BITRATE_500K_1M]   = CAN_500K_1M,
    [HAL_CAN_BITRATE_500K_2M]   = CAN_500K_2M,
    [HAL_CAN_BITRATE_500K_4M]   = CAN_500K_4M,
    [HAL_CAN_BITRATE_500K_5M]   = CAN_500K_5M,
    [HAL_CAN_BITRATE_500K_8M]   = CAN_500K_8M,
    [HAL_CAN_BITRATE_1000K_4M]  = CAN_1000K_4M,
    [HAL_CAN_BITRATE_1000K_8M]  = CAN_1000K_8M,
};

static const CAN_OPERATION_MODE mcp_mode_map[HAL_CAN_MODE_MAX] = {
    [HAL_CAN_MODE_NORMAL]            = CAN_NORMAL_MODE,
    [HAL_CAN_MODE_CLASSIC]           = CAN_CLASSIC_MODE,
    [HAL_CAN_MODE_LISTEN_ONLY]       = CAN_LISTEN_ONLY_MODE,
    [HAL_CAN_MODE_INTERNAL_LOOPBACK] = CAN_INTERNAL_LOOPBACK_MODE,
    [HAL_CAN_MODE_EXTERNAL_LOOPBACK] = CAN_EXTERNAL_LOOPBACK_MODE,
};

/* FD payload sizes for DLC 9..15 (DLC 0..8 map 1:1). */
static const uint8_t mcp_fd_sizes[7] = { 12, 16, 20, 24, 32, 48, 64 };

/* ---- helpers --------------------------------------------------------------- */

static bool mcp_timed_out(uint32_t start_ms, size_t timeout_ms)
{
    return (size_t)(hal_get_tick_ms() - start_ms) >= timeout_ms;
}

/* Wire length (already validated by hal_can) -> DLC. Not the vendor's
 * DRV_CANFDSPI_DataBytesToDlc(): it rounds 0..3 up to DLC 4 and 5..7 to DLC 8. */
static CAN_DLC mcp_len_to_dlc(uint8_t len)
{
    uint8_t i = 0;

    if (len <= 8u) {
        return (CAN_DLC)len;
    }
    for (i = 0; i < sizeof(mcp_fd_sizes) - 1u; i++) {
        if (len <= mcp_fd_sizes[i]) {
            break;
        }
    }
    return (CAN_DLC)(CAN_DLC_12 + i);
}

static uint8_t mcp_dlc_to_len(uint32_t dlc, bool fd)
{
    if (dlc <= 8u) {
        return (uint8_t)dlc;
    }
    /* Classic frames: DLC 9..15 still carry 8 bytes. */
    return fd ? mcp_fd_sizes[(dlc - 9u) & 0x7u] : 8u;
}

static int32_t mcp_wait_osc_ready(void)
{
    CAN_OSC_STATUS status = { 0 };
    uint32_t start = hal_get_tick_ms();

    do {
        if (DRV_CANFDSPI_OscillatorStatusGet(&status) == 0 && status.OscReady) {
            return HAL_SUCCESS;
        }
    } while (!mcp_timed_out(start, MCP_OSC_READY_TIMEOUT_MS));

    return HAL_ERR_TIMEOUT;
}

/* A controller answers if two complementary patterns written to message RAM
 * read back unchanged (a floating or shorted MISO cannot fake both). */
static int32_t mcp_probe(void)
{
    static const uint32_t patterns[2] = { 0x5AA5C33Cu, 0xA55A3CC3u };
    uint32_t readback = 0;
    uint8_t i = 0;

    for (i = 0; i < 2u; i++) {
        if (DRV_CANFDSPI_WriteWord(cRAMADDR_START, patterns[i]) != 0 ||
            DRV_CANFDSPI_ReadWord(cRAMADDR_START, &readback) != 0 ||
            readback != patterns[i]) {
            return HAL_ERR_COMMUNICATION;
        }
    }
    return HAL_SUCCESS;
}

static int32_t mcp_set_mode(CAN_OPERATION_MODE mode)
{
    uint32_t start = 0;

    if (DRV_CANFDSPI_OperationModeSelect(mode) != 0) {
        return HAL_ERR_HW_CONTROL;
    }
    start = hal_get_tick_ms();
    do {
        if (DRV_CANFDSPI_OperationModeGet() == mode) {
            return HAL_SUCCESS;
        }
    } while (!mcp_timed_out(start, MCP_MODE_SWITCH_TIMEOUT_MS));

    return HAL_ERR_HW_STATE;
}

static int32_t mcp_setup_filter(const HAL_CAN *hcan)
{
    CAN_FILTEROBJ_ID fobj;
    CAN_MASKOBJ_ID mobj;

    memset(&fobj, 0, sizeof(fobj));
    memset(&mobj, 0, sizeof(mobj));

    if (hcan->rx_filter_mask != 0u) {
        if (hcan->rx_filter_extended) {
            fobj.SID = (hcan->rx_filter_id >> 18) & 0x7FFu;
            fobj.EID = hcan->rx_filter_id & 0x3FFFFu;
            fobj.EXIDE = 1;
            mobj.MSID = (hcan->rx_filter_mask >> 18) & 0x7FFu;
            mobj.MEID = hcan->rx_filter_mask & 0x3FFFFu;
        } else {
            fobj.SID = hcan->rx_filter_id & 0x7FFu;
            mobj.MSID = hcan->rx_filter_mask & 0x7FFu;
        }
        mobj.MIDE = 1;      /* ID type must match EXIDE */
    }
    /* mask == 0: all-zero filter + mask (MIDE = 0) accepts everything. */

    if (DRV_CANFDSPI_FilterObjectConfigure(MCP_RX_FILTER, &fobj) != 0 ||
        DRV_CANFDSPI_FilterMaskConfigure(MCP_RX_FILTER, &mobj) != 0 ||
        DRV_CANFDSPI_FilterToFifoLink(MCP_RX_FILTER, MCP_RX_FIFO, true) != 0) {
        return HAL_ERR_INIT;
    }
    return HAL_SUCCESS;
}

static int32_t mcp_configure(const HAL_CAN *hcan)
{
    CAN_CONFIG config;
    CAN_TX_FIFO_CONFIG tx_config;
    CAN_RX_FIFO_CONFIG rx_config;

    if (DRV_CANFDSPI_EccEnable() != 0 || DRV_CANFDSPI_RamInit(0xFF) != 0) {
        return HAL_ERR_INIT;
    }

    DRV_CANFDSPI_ConfigureObjectReset(&config);
    config.IsoCrcEnable = 1;
    config.StoreInTEF = 0;
    config.TXQEnable = 0;
    if (DRV_CANFDSPI_Configure(&config) != 0) {
        return HAL_ERR_INIT;
    }

    DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&tx_config);
    tx_config.FifoSize = MCP_TX_FIFO_DEPTH - 1u;
    tx_config.PayLoadSize = CAN_PLSIZE_64;
    tx_config.TxPriority = 1;
    if (DRV_CANFDSPI_TransmitChannelConfigure(MCP_TX_FIFO, &tx_config) != 0) {
        return HAL_ERR_INIT;
    }

    DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rx_config);
    rx_config.FifoSize = MCP_RX_FIFO_DEPTH - 1u;
    rx_config.PayLoadSize = CAN_PLSIZE_64;
    if (DRV_CANFDSPI_ReceiveChannelConfigure(MCP_RX_FIFO, &rx_config) != 0) {
        return HAL_ERR_INIT;
    }

    if (mcp_setup_filter(hcan) != HAL_SUCCESS) {
        return HAL_ERR_INIT;
    }

    if (DRV_CANFDSPI_BitTimeConfigure(mcp_bitrate_map[hcan->bitrate], CAN_SSP_MODE_AUTO,
                                      MCP_SYSCLK) != 0) {
        /* The vendor tables reject rate/oscillator combinations they cannot time. */
        return HAL_ERR_INVALID_PARAM;
    }

    if (DRV_CANFDSPI_GpioModeConfigure(GPIO_MODE_INT, GPIO_MODE_INT) != 0 ||
        DRV_CANFDSPI_TransmitChannelEventEnable(MCP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT) != 0 ||
        DRV_CANFDSPI_ReceiveChannelEventEnable(MCP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT) != 0 ||
        DRV_CANFDSPI_ModuleEventEnable(CAN_TX_EVENT | CAN_RX_EVENT) != 0) {
        return HAL_ERR_INIT;
    }

    return HAL_SUCCESS;
}

/* ---- backend ops ------------------------------------------------------------ */

static int32_t mcp_init(const HAL_CAN *hcan)
{
    int32_t ret = HAL_SUCCESS;
    uint32_t spi_hz = MCP_SPI_LIMIT_HZ;

#if HAL_CONFIG_CAN1_MCP2518FD_SPI_MAX_HZ != 0
    if (HAL_CONFIG_CAN1_MCP2518FD_SPI_MAX_HZ < spi_hz) {
        spi_hz = HAL_CONFIG_CAN1_MCP2518FD_SPI_MAX_HZ;
    }
#endif

    ret = drv_canfdspi_port_init(spi_hz, NULL);
    if (ret != HAL_SUCCESS) {
        return ret;
    }

    /* Reset puts the controller in configuration mode. */
    if (DRV_CANFDSPI_Reset() != 0) {
        ret = HAL_ERR_COMMUNICATION;
        goto fail;
    }
    if (mcp_wait_osc_ready() != HAL_SUCCESS) {
        /* No oscillator -- or no controller at all (MISO reads garbage). */
        ret = (mcp_probe() != HAL_SUCCESS) ? HAL_ERR_COMMUNICATION : HAL_ERR_TIMEOUT;
        goto fail;
    }
    ret = mcp_probe();
    if (ret != HAL_SUCCESS) {
        goto fail;
    }

    ret = mcp_configure(hcan);
    if (ret != HAL_SUCCESS) {
        goto fail;
    }

    ret = mcp_set_mode(mcp_mode_map[hcan->mode]);
    if (ret != HAL_SUCCESS) {
        goto fail;
    }
    return HAL_SUCCESS;

fail:
    drv_canfdspi_port_deinit();
    return ret;
}

static void mcp_deinit(void)
{
    (void)mcp_set_mode(CAN_CONFIGURATION_MODE);
    drv_canfdspi_port_deinit();
}

static int32_t mcp_send(const HAL_CAN_MSG *msg, size_t timeout_ms)
{
    CAN_TX_MSGOBJ obj;
    CAN_TX_FIFO_EVENT flags = CAN_TX_FIFO_NO_EVENT;
    uint32_t start = 0;

    memset(&obj, 0, sizeof(obj));
    if (msg->extended) {
        obj.bF.id.SID = (msg->id >> 18) & 0x7FFu;
        obj.bF.id.EID = msg->id & 0x3FFFFu;
    } else {
        obj.bF.id.SID = msg->id & 0x7FFu;
    }
    obj.bF.ctrl.DLC = mcp_len_to_dlc(msg->len);
    obj.bF.ctrl.IDE = msg->extended ? 1u : 0u;
    obj.bF.ctrl.FDF = msg->fd ? 1u : 0u;
    obj.bF.ctrl.BRS = msg->brs ? 1u : 0u;

    start = hal_get_tick_ms();
    for (;;) {
        if (DRV_CANFDSPI_TransmitChannelEventGet(MCP_TX_FIFO, &flags) != 0) {
            return HAL_ERR_HW_SEND;
        }
        if (flags & CAN_TX_FIFO_NOT_FULL_EVENT) {
            break;
        }
        if (mcp_timed_out(start, timeout_ms)) {
            return HAL_ERR_TIMEOUT;
        }
    }

    /* msg->len is the wire length, i.e. exactly the DLC size. */
    if (DRV_CANFDSPI_TransmitChannelLoad(MCP_TX_FIFO, &obj, (uint8_t *)msg->data,
                                         msg->len, true) != 0) {
        return HAL_ERR_HW_SEND;
    }
    return HAL_SUCCESS;
}

static int32_t mcp_receive(HAL_CAN_MSG *msg, size_t timeout_ms)
{
    CAN_RX_MSGOBJ obj;
    CAN_RX_FIFO_EVENT flags = CAN_RX_FIFO_NO_EVENT;
    uint32_t start = hal_get_tick_ms();

    for (;;) {
        if (DRV_CANFDSPI_ReceiveChannelEventGet(MCP_RX_FIFO, &flags) != 0) {
            return HAL_ERR_HW_RECV;
        }
        if (flags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
            break;
        }
        if (mcp_timed_out(start, timeout_ms)) {
            return HAL_ERR_TIMEOUT;
        }
    }

    memset(&obj, 0, sizeof(obj));
    if (DRV_CANFDSPI_ReceiveMessageGet(MCP_RX_FIFO, &obj, msg->data, HAL_CAN_MAX_DATA_LEN) != 0) {
        return HAL_ERR_HW_RECV;
    }

    msg->extended = obj.bF.ctrl.IDE != 0u;
    msg->id = msg->extended ? (((uint32_t)obj.bF.id.SID << 18) | obj.bF.id.EID)
                            : obj.bF.id.SID;
    msg->fd = obj.bF.ctrl.FDF != 0u;
    msg->brs = obj.bF.ctrl.BRS != 0u;
    msg->len = mcp_dlc_to_len(obj.bF.ctrl.DLC, msg->fd);
    return HAL_SUCCESS;
}

static int32_t mcp_get_error_state(HAL_CAN_ERROR_STATE *state)
{
    CAN_ERROR_STATE flags = CAN_ERROR_FREE_STATE;

    if (DRV_CANFDSPI_ErrorCountStateGet(&state->tec, &state->rec, &flags) != 0) {
        return HAL_ERR_HW_READ;
    }
    state->error_passive = (flags & (CAN_TX_BUS_PASSIVE_STATE | CAN_RX_BUS_PASSIVE_STATE)) != 0;
    state->bus_off = (flags & CAN_TX_BUS_OFF_STATE) != 0;
    return HAL_SUCCESS;
}

const HAL_CAN_BACKEND hal_can_backend_mcp2518fd = {
    .name = "mcp2518fd",
    .init = mcp_init,
    .deinit = mcp_deinit,
    .send = mcp_send,
    .receive = mcp_receive,
    .get_error_state = mcp_get_error_state,
};

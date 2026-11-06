#include <string.h>
#include "hal_spi.h"
#include "hal_err.h"
#include "stm32l552xx.h"
#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_spi.h"
#include "stm32l5xx_hal_gpio_ex.h"

/*
 * HAL_SPI_1 -> SPI1 on NUCLEO-L552ZE-Q
 *   PA5  ------> SPI1_SCK   (AF5, CN7 pin 10)
 *   PA6  ------> SPI1_MISO  (AF5, CN7 pin 12)
 *   PA7  ------> SPI1_MOSI  (AF5, CN7 pin 14)
 *   PD14 ------> CS         (GPIO output, active low)
 */
#define SPI1_BUS_PORT           GPIOA
#define SPI1_BUS_PINS           (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define SPI1_CS_PORT            GPIOD
#define SPI1_CS_PIN             GPIO_PIN_14

/* HAL_SPI_Transmit/Receive take a uint16_t length. */
#define SPI_HAL_MAX_CHUNK       (0xFFFFu)
/* Buffer used to clock out dummy bytes when the caller passes neither tx nor rx. */
#define SPI_DUMMY_CHUNK         (32u)

static SPI_HandleTypeDef hspi1;     // map to HAL_SPI_1
static bool hspi1_ready = false;

static const uint8_t spi_dummy_tx[SPI_DUMMY_CHUNK] = { 0 };

static int32_t stm32l5xx_spi_status_to_hal_err(HAL_StatusTypeDef status)
{
    switch (status) {
        case HAL_OK:      return HAL_SUCCESS;
        case HAL_BUSY:    return HAL_ERR_BUSY;
        case HAL_TIMEOUT: return HAL_ERR_TIMEOUT;
        default:          return HAL_ERR_COMMUNICATION;
    }
}

/*
 * Pick the fastest prescaler (PCLK / 2 .. PCLK / 256) whose SCK does not
 * exceed max_hz. max_hz == 0 or a limit below PCLK/256 selects /256.
 */
static uint32_t stm32l5xx_spi_pick_prescaler(uint32_t pclk_hz, uint32_t max_hz,
                                             uint32_t *speed_hz)
{
    uint32_t n = 0;

    for (n = 0; n < 7u; n++) {
        if (max_hz != 0u && (pclk_hz >> (n + 1u)) <= max_hz) {
            break;
        }
    }
    *speed_hz = pclk_hz >> (n + 1u);

    /* BR[2:0] in CR1 selects PCLK / 2^(BR+1); same encoding as SPI_BAUDRATEPRESCALER_x. */
    return n << SPI_CR1_BR_Pos;
}

static uint32_t stm32l5xx_ms_left(uint32_t start_tick, size_t timeout_ms)
{
    uint32_t elapsed = HAL_GetTick() - start_tick;

    return (elapsed >= timeout_ms) ? 0u : (uint32_t)(timeout_ms - elapsed);
}

static void stm32l5xx_spi1_cs_assert(void)
{
    HAL_GPIO_WritePin(SPI1_CS_PORT, SPI1_CS_PIN, GPIO_PIN_RESET);
}

static void stm32l5xx_spi1_cs_release(void)
{
    HAL_GPIO_WritePin(SPI1_CS_PORT, SPI1_CS_PIN, GPIO_PIN_SET);
}

static int32_t stm32l5xx_spi1_init_map_to_hal_spi_1(HAL_SPI *hspi)
{
    int32_t ret = HAL_SUCCESS;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t prescaler = 0;

    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* CS: drive high before switching the pin to output so the slave never
     * sees a spurious select. */
    HAL_GPIO_WritePin(SPI1_CS_PORT, SPI1_CS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SPI1_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI1_CS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI1_BUS_PINS;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(SPI1_BUS_PORT, &GPIO_InitStruct);

    /* SPI1 sits on APB2: its kernel clock is PCLK2. */
    prescaler = stm32l5xx_spi_pick_prescaler(HAL_RCC_GetPCLK2Freq(),
                                             hspi->max_speed_hz,
                                             &hspi->speed_hz);

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = (hspi->mode & 0x2u) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = (hspi->mode & 0x1u) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = prescaler;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        ret = HAL_ERR_INIT;
        return ret;
    }

    hspi1_ready = true;
    return ret;
}

static void stm32l5xx_spi1_deinit_map_to_hal_spi_1(void)
{
    HAL_SPI_DeInit(&hspi1);
    HAL_GPIO_DeInit(SPI1_BUS_PORT, SPI1_BUS_PINS);
    HAL_GPIO_DeInit(SPI1_CS_PORT, SPI1_CS_PIN);
    __HAL_RCC_SPI1_CLK_DISABLE();
    hspi1_ready = false;
}

/*
 * One phase of a transfer, CS already asserted. Splits into HAL-sized chunks.
 */
static int32_t stm32l5xx_spi1_xfer_map_to_hal_spi_1(const uint8_t *tx, uint8_t *rx,
                                                    size_t len, uint32_t start_tick,
                                                    size_t timeout_ms)
{
    int32_t ret = HAL_SUCCESS;
    HAL_StatusTypeDef status = HAL_OK;
    size_t done = 0;

    while (done < len) {
        size_t chunk = len - done;
        uint32_t left = stm32l5xx_ms_left(start_tick, timeout_ms);

        if (left == 0u) {
            ret = HAL_ERR_TIMEOUT;
            break;
        }

        if (tx != NULL && rx != NULL) {
            if (chunk > SPI_HAL_MAX_CHUNK) {
                chunk = SPI_HAL_MAX_CHUNK;
            }
            /* tx == rx is fine: each byte is read out before its slot is overwritten. */
            status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx[done], &rx[done],
                                             (uint16_t)chunk, left);
        } else if (tx != NULL) {
            if (chunk > SPI_HAL_MAX_CHUNK) {
                chunk = SPI_HAL_MAX_CHUNK;
            }
            status = HAL_SPI_Transmit(&hspi1, (uint8_t *)&tx[done], (uint16_t)chunk, left);
        } else if (rx != NULL) {
            if (chunk > SPI_HAL_MAX_CHUNK) {
                chunk = SPI_HAL_MAX_CHUNK;
            }
            /* Fill with dummy bytes and exchange in place: what goes out on MOSI
             * is always HAL_SPI_DUMMY_BYTE, not stale buffer contents. */
            memset(&rx[done], HAL_SPI_DUMMY_BYTE, chunk);
            status = HAL_SPI_TransmitReceive(&hspi1, &rx[done], &rx[done],
                                             (uint16_t)chunk, left);
        } else {
            if (chunk > SPI_DUMMY_CHUNK) {
                chunk = SPI_DUMMY_CHUNK;
            }
            status = HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_dummy_tx, (uint16_t)chunk, left);
        }

        ret = stm32l5xx_spi_status_to_hal_err(status);
        if (ret != HAL_SUCCESS) {
            break;
        }
        done += chunk;
    }

    return ret;
}

int32_t hal_spi_init(HAL_SPI *hspi)
{
    int32_t ret = HAL_SUCCESS;

    if (hspi == NULL) {
        ret = HAL_ERR_NULL_POINTER;
        goto finish;
    }

    if (hspi->mode >= HAL_SPI_MODE_MAX) {
        ret = HAL_ERR_INVALID_PARAM;
        goto finish;
    }

    switch (hspi->num) {
        case HAL_SPI_1:
            ret = stm32l5xx_spi1_init_map_to_hal_spi_1(hspi);
            break;
        default:
            ret = HAL_ERR_HW_LACK;
            break;
    }

finish:
    return ret;
}

void hal_spi_deinit(HAL_SPI *hspi)
{
    if (hspi == NULL) {
        return;
    }

    switch (hspi->num) {
        case HAL_SPI_1:
            stm32l5xx_spi1_deinit_map_to_hal_spi_1();
            break;
        default:
            break;
    }
}

/* Common guard: bus mapped on this board and initialized. */
static int32_t stm32l5xx_spi_check_ready(HAL_SPI *hspi)
{
    if (hspi == NULL) {
        return HAL_ERR_NULL_POINTER;
    }

    switch (hspi->num) {
        case HAL_SPI_1:
            return hspi1_ready ? HAL_SUCCESS : HAL_ERR_NOT_INITIALIZED;
        default:
            return HAL_ERR_HW_LACK;
    }
}

int32_t hal_spi_cs_assert(HAL_SPI *hspi)
{
    int32_t ret = stm32l5xx_spi_check_ready(hspi);

    if (ret == HAL_SUCCESS) {
        stm32l5xx_spi1_cs_assert();     /* only HAL_SPI_1 passes the check */
    }
    return ret;
}

int32_t hal_spi_cs_release(HAL_SPI *hspi)
{
    int32_t ret = stm32l5xx_spi_check_ready(hspi);

    if (ret == HAL_SUCCESS) {
        stm32l5xx_spi1_cs_release();
    }
    return ret;
}

int32_t hal_spi_transfer_raw(HAL_SPI *hspi, const uint8_t *tx, uint8_t *rx,
                             size_t len, size_t timeout_ms)
{
    int32_t ret = stm32l5xx_spi_check_ready(hspi);

    if (ret != HAL_SUCCESS || len == 0u) {
        return ret;
    }
    return stm32l5xx_spi1_xfer_map_to_hal_spi_1(tx, rx, len, HAL_GetTick(), timeout_ms);
}

int32_t hal_spi_transfer(HAL_SPI *hspi, const uint8_t *tx, uint8_t *rx,
                         size_t len, size_t timeout_ms)
{
    int32_t ret = stm32l5xx_spi_check_ready(hspi);

    if (ret != HAL_SUCCESS || len == 0u) {
        return ret;
    }

    stm32l5xx_spi1_cs_assert();
    ret = stm32l5xx_spi1_xfer_map_to_hal_spi_1(tx, rx, len, HAL_GetTick(), timeout_ms);
    stm32l5xx_spi1_cs_release();
    return ret;
}

int32_t hal_spi_write_then_read(HAL_SPI *hspi, const uint8_t *tx, size_t tx_len,
                                uint8_t *rx, size_t rx_len, size_t timeout_ms)
{
    int32_t ret = HAL_SUCCESS;

    if ((tx == NULL && tx_len != 0u) || (rx == NULL && rx_len != 0u)) {
        return HAL_ERR_NULL_POINTER;
    }

    ret = stm32l5xx_spi_check_ready(hspi);
    if (ret != HAL_SUCCESS || (tx_len == 0u && rx_len == 0u)) {
        return ret;
    }

    stm32l5xx_spi1_cs_assert();
    ret = stm32l5xx_spi1_xfer_map_to_hal_spi_1(tx, NULL, tx_len, HAL_GetTick(), timeout_ms);
    if (ret == HAL_SUCCESS) {
        ret = stm32l5xx_spi1_xfer_map_to_hal_spi_1(NULL, rx, rx_len, HAL_GetTick(), timeout_ms);
    }
    stm32l5xx_spi1_cs_release();
    return ret;
}

#if HAL_UNIT_TEST
/*
 * Loopback test: jumper PA7 (MOSI) to PA6 (MISO), nothing else on the bus.
 * Whatever is shifted out comes straight back in.
 */
int32_t hal_spi_unit_test(void)
{
    int32_t ret = HAL_SUCCESS;
    HAL_SPI spi = {
        .num = HAL_SPI_1,
        .mode = HAL_SPI_MODE_0,
        .max_speed_hz = 1000000u,
    };
    const uint8_t pattern[8] = { 0xA5, 0x5A, 0x00, 0xFF, 0x01, 0x80, 0x3C, 0xC3 };
    uint8_t rx[8] = { 0 };
    uint8_t buf[8] = { 0 };
    size_t timeout_ms = 10;

    /* Parameter checks need no hardware. */
    if (hal_spi_init(NULL) != HAL_ERR_NULL_POINTER) {
        return HAL_ERR;
    }
    if (hal_spi_transfer(&spi, pattern, rx, sizeof(rx), timeout_ms) != HAL_ERR_NOT_INITIALIZED) {
        return HAL_ERR;
    }

    ret = hal_spi_init(&spi);
    if (ret != HAL_SUCCESS) {
        return ret;
    }
    /* Fastest SCK not above 1 MHz. */
    if (spi.speed_hz == 0u || spi.speed_hz > spi.max_speed_hz) {
        ret = HAL_ERR;
        goto out;
    }

    /* 1. Full duplex: rx must equal what was sent. */
    ret = hal_spi_transfer(&spi, pattern, rx, sizeof(pattern), timeout_ms);
    if (ret != HAL_SUCCESS) {
        goto out;
    }
    if (memcmp(pattern, rx, sizeof(pattern)) != 0) {
        ret = HAL_ERR_COMMUNICATION;
        goto out;
    }

    /* 2. Pure read: dummy bytes go out, so dummy bytes come back. */
    memset(rx, 0x77, sizeof(rx));
    ret = hal_spi_transfer(&spi, NULL, rx, sizeof(rx), timeout_ms);
    if (ret != HAL_SUCCESS) {
        goto out;
    }
    for (size_t i = 0; i < sizeof(rx); i++) {
        if (rx[i] != HAL_SPI_DUMMY_BYTE) {
            ret = HAL_ERR_COMMUNICATION;
            goto out;
        }
    }

    /* 3. In-place exchange. */
    memcpy(buf, pattern, sizeof(buf));
    ret = hal_spi_transfer(&spi, buf, buf, sizeof(buf), timeout_ms);
    if (ret != HAL_SUCCESS) {
        goto out;
    }
    if (memcmp(pattern, buf, sizeof(pattern)) != 0) {
        ret = HAL_ERR_COMMUNICATION;
        goto out;
    }

    /* 4. Header + reply in one CS window (reply phase sends dummies). */
    memset(rx, 0x77, sizeof(rx));
    ret = hal_spi_write_then_read(&spi, pattern, 2, rx, 4, timeout_ms);
    if (ret != HAL_SUCCESS) {
        goto out;
    }
    if (rx[0] != HAL_SPI_DUMMY_BYTE || rx[3] != HAL_SPI_DUMMY_BYTE || rx[4] != 0x77) {
        ret = HAL_ERR_COMMUNICATION;
        goto out;
    }

out:
    hal_spi_deinit(&spi);
    return ret;
}
#endif /* HAL_UNIT_TEST */

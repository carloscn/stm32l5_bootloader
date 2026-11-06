/*
 * STM32 port layer for Microchip's drv_canfdspi driver: maps the vendor's
 * SPI/CS hooks onto hal_spi. See drv_canfdspi_port.h.
 *
 * Do not include any stm32l5xx_*.h here (see the header for why).
 */
#include "drv_canfdspi_port.h"
#include "hal_spi.h"
#include "hal_err.h"

static HAL_SPI canfd_spi = {
    .num = HAL_SPI_1,
    .mode = HAL_SPI_MODE_0,       /* MCP2518FD supports SPI mode 0,0 and 1,1 */
    .max_speed_hz = 0,
    .speed_hz = 0,
};

static int32_t canfd_spi_last_err = HAL_SUCCESS;

static void port_note(int32_t ret)
{
    if (ret != HAL_SUCCESS) {
        canfd_spi_last_err = ret;
    }
}

int32_t drv_canfdspi_port_init(uint32_t max_speed_hz, uint32_t *speed_hz)
{
    int32_t ret = HAL_SUCCESS;

    canfd_spi_last_err = HAL_SUCCESS;
    canfd_spi.max_speed_hz = max_speed_hz;

    ret = hal_spi_init(&canfd_spi);
    if (ret == HAL_SUCCESS && speed_hz != NULL) {
        *speed_hz = canfd_spi.speed_hz;
    }
    port_note(ret);
    return ret;
}

void drv_canfdspi_port_deinit(void)
{
    hal_spi_deinit(&canfd_spi);
}

int32_t drv_canfdspi_port_last_error(void)
{
    return canfd_spi_last_err;
}

void CAN_CS_Clear(void)
{
    port_note(hal_spi_cs_assert(&canfd_spi));
}

void CAN_CS_Set(void)
{
    port_note(hal_spi_cs_release(&canfd_spi));
}

bool SERCOM1_SPI_WriteRead(void *pTransmitData, size_t txSize,
                           void *pReceiveData, size_t rxSize)
{
    int32_t ret = HAL_SUCCESS;

    if (txSize != 0u) {
        if (pTransmitData == NULL) {
            port_note(HAL_ERR_NULL_POINTER);
            return false;
        }
        ret = hal_spi_transfer_raw(&canfd_spi, (const uint8_t *)pTransmitData, NULL,
                                   txSize, DRV_CANFDSPI_PORT_SPI_TIMEOUT_MS);
        if (ret != HAL_SUCCESS) {
            port_note(ret);
            return false;
        }
    }

    if (rxSize != 0u) {
        if (pReceiveData == NULL) {
            port_note(HAL_ERR_NULL_POINTER);
            return false;
        }
        ret = hal_spi_transfer_raw(&canfd_spi, NULL, (uint8_t *)pReceiveData,
                                   rxSize, DRV_CANFDSPI_PORT_SPI_TIMEOUT_MS);
        if (ret != HAL_SUCCESS) {
            port_note(ret);
            return false;
        }
    }

    return true;
}

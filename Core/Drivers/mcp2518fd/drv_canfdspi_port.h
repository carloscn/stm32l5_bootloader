/*
 * STM32 port layer for Microchip's MCP25xxFD "drv_canfdspi" driver.
 *
 * Origin of drv_canfdspi_api.[ch], drv_canfdspi_defines.h and
 * drv_canfdspi_register.h: Microchip's MCP2518FD example as published in
 * https://github.com/ChaoA51933/mcp2518fd-based-on-atsamd21g17d
 * (firmware/src/driver/mcp25xxfd_driver). Microchip license terms are kept in
 * each file header. Changes to those files (both in drv_canfdspi_api.c):
 *   1. includes: definitions.h + <xc.h> -> this header;
 *   2. DRV_CANFDSPI_TransmitChannelLoad(): the zero padding up to a 4-byte
 *      boundary was written 8 bytes past the payload (txBuffer[i + 8 + j] with
 *      i already = len + 8), leaving the real padding uninitialized.
 *
 * The vendor code talks to the bus through exactly three symbols, mapped here
 * onto hal_spi (SPI1, CS = PD14 on NUCLEO-L552ZE-Q):
 *     CAN_CS_Clear()            -> hal_spi_cs_assert()
 *     CAN_CS_Set()              -> hal_spi_cs_release()
 *     SERCOM1_SPI_WriteRead()   -> hal_spi_transfer_raw()
 *
 * IMPORTANT: drv_canfdspi_defines.h declares enum constants GPIO_PIN_0 and
 * GPIO_PIN_1, which clash with STM32 HAL macros of the same name. Never include
 * the drv_canfdspi_*.h headers from a file that also includes stm32l5xx_*.h.
 */
#ifndef _DRV_CANFDSPI_PORT_H_
#define _DRV_CANFDSPI_PORT_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Per-transfer SPI timeout; the longest vendor transfer is ~100 bytes. */
#define DRV_CANFDSPI_PORT_SPI_TIMEOUT_MS    (10u)

/**
 * @brief Bring up the SPI bus used by the driver.
 *
 * @param max_speed_hz SCK upper limit. The MCP2518FD allows at most
 *                     0.85 * SYSCLK / 2 (17 MHz with a 40 MHz oscillator).
 * @param speed_hz     out (optional): actual SCK frequency.
 * @return int32_t HAL_SUCCESS or a hal_err.h code from hal_spi_init().
 */
int32_t drv_canfdspi_port_init(uint32_t max_speed_hz, uint32_t *speed_hz);

/** @brief Release the SPI bus. */
void drv_canfdspi_port_deinit(void);

/**
 * @brief Last hal_spi error seen by the port (HAL_SUCCESS if none).
 *
 * The vendor API only reports pass/fail (int8_t 0 / -1); this keeps the real
 * cause for diagnostics. Cleared by drv_canfdspi_port_init().
 */
int32_t drv_canfdspi_port_last_error(void);

/* ---- symbols used by drv_canfdspi_api.c ---------------------------------- */

void CAN_CS_Clear(void);
void CAN_CS_Set(void);

/**
 * Same contract as Harmony's SERCOMx_SPI_WriteRead(): sends txSize bytes, then
 * receives rxSize bytes (dummy bytes are clocked out meanwhile). CS is not
 * touched. Returns true on success.
 */
bool SERCOM1_SPI_WriteRead(void *pTransmitData, size_t txSize,
                           void *pReceiveData, size_t rxSize);

#ifndef Nop
#define Nop()   __asm volatile ("nop")
#endif

#endif /* _DRV_CANFDSPI_PORT_H_ */

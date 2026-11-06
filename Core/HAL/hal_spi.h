#ifndef _HAL_SPI_H_
#define _HAL_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "hal_config.h"

/*
 * SPI master HAL.
 *
 * Every transfer is framed by the chip-select line: CS is asserted before the
 * first byte and released after the last one, so one call == one SPI
 * transaction on the bus (this is what devices such as the MCP2518FD expect:
 * <4-bit cmd + 12-bit addr><data...> inside a single CS window).
 *
 * SPI is full duplex: the master clocks one byte in for every byte it clocks
 * out. To read, the HAL shifts out dummy bytes (HAL_SPI_DUMMY_BYTE) and keeps
 * what arrives on MISO.
 *
 * Board mapping (NUCLEO-L552ZE-Q):
 *   HAL_SPI_1 -> SPI1  SCK=PA5  MISO=PA6  MOSI=PA7 (CN7 pins 10/12/14)
 *                      CS =PD14 (GPIO, active low)
 */

#define HAL_SPI_DUMMY_BYTE      (0x00u)

typedef enum hal_spi_num {
    HAL_SPI_1 = 0x0,
    HAL_SPI_NUM_MAX
} HAL_SPI_NUM;

/* Clock polarity / phase, standard Motorola numbering. */
typedef enum hal_spi_mode {
    HAL_SPI_MODE_0 = 0x0,   /* CPOL=0, CPHA=0 (sample on rising edge)  */
    HAL_SPI_MODE_1,         /* CPOL=0, CPHA=1                          */
    HAL_SPI_MODE_2,         /* CPOL=1, CPHA=0                          */
    HAL_SPI_MODE_3,         /* CPOL=1, CPHA=1                          */
    HAL_SPI_MODE_MAX
} HAL_SPI_MODE;

typedef struct hal_spi_t {
    HAL_SPI_NUM  num;           /* which SPI bus                                   */
    HAL_SPI_MODE mode;          /* CPOL/CPHA                                       */
    uint32_t     max_speed_hz;  /* upper limit for SCK; the HAL picks the fastest
                                   peripheral prescaler that does not exceed it.
                                   0 selects the slowest possible clock.           */
    uint32_t     speed_hz;      /* out: actual SCK after hal_spi_init()            */
} HAL_SPI;

/**
 * @brief Initialize an SPI bus as master (8-bit, MSB first, software CS).
 *
 * Configures the bus pins, the CS pin (idle high) and the peripheral clock
 * divider. On success hspi->speed_hz holds the real SCK frequency.
 *
 * @param hspi Pointer to the HAL_SPI context (num/mode/max_speed_hz filled in).
 * @return int32_t HAL_SUCCESS, HAL_ERR_NULL_POINTER, HAL_ERR_INVALID_PARAM,
 *                 HAL_ERR_HW_LACK (bus not mapped on this board) or HAL_ERR_INIT.
 */
int32_t hal_spi_init(HAL_SPI *hspi);

/**
 * @brief Deinitialize an SPI bus and release its pins.
 *
 * @param hspi Pointer to the HAL_SPI context.
 */
void hal_spi_deinit(HAL_SPI *hspi);

/**
 * @brief Full-duplex transfer inside one CS window.
 *
 * Clocks out len bytes from tx while storing len bytes from MISO into rx.
 *  - tx == NULL: HAL_SPI_DUMMY_BYTE is sent for every byte (pure read).
 *  - rx == NULL: received bytes are discarded (pure write).
 *  - tx == rx is allowed (in-place exchange).
 *
 * @param hspi       Pointer to an initialized HAL_SPI context.
 * @param tx         Bytes to send, or NULL.
 * @param rx         Buffer for received bytes, or NULL.
 * @param len        Number of bytes to exchange (0 is a no-op).
 * @param timeout_ms Timeout for the whole transfer.
 * @return int32_t HAL_SUCCESS, HAL_ERR_NULL_POINTER, HAL_ERR_NOT_INITIALIZED,
 *                 HAL_ERR_BUSY, HAL_ERR_TIMEOUT or HAL_ERR_COMMUNICATION.
 */
int32_t hal_spi_transfer(HAL_SPI *hspi, const uint8_t *tx, uint8_t *rx,
                         size_t len, size_t timeout_ms);

/**
 * @brief Write a header, then read a reply, inside one CS window.
 *
 * Typical register read: tx = command/address bytes, rx = register contents.
 * Bytes clocked in while tx is being sent are discarded; dummy bytes are sent
 * while rx is being filled.
 *
 * @param hspi       Pointer to an initialized HAL_SPI context.
 * @param tx         Header bytes to send (may be NULL only if tx_len == 0).
 * @param tx_len     Number of header bytes.
 * @param rx         Buffer for the reply (may be NULL only if rx_len == 0).
 * @param rx_len     Number of reply bytes.
 * @param timeout_ms Timeout applied to each of the two phases.
 * @return int32_t Same codes as hal_spi_transfer().
 */
int32_t hal_spi_write_then_read(HAL_SPI *hspi, const uint8_t *tx, size_t tx_len,
                                uint8_t *rx, size_t rx_len, size_t timeout_ms);

/*
 * Manual chip-select control, for drivers that build one bus transaction out
 * of several transfers (e.g. vendor code that sends a header and then reads in
 * a second call):
 *
 *     hal_spi_cs_assert(&spi);
 *     hal_spi_transfer_raw(&spi, hdr, NULL, 2, 10);
 *     hal_spi_transfer_raw(&spi, NULL, data, n, 10);
 *     hal_spi_cs_release(&spi);
 *
 * hal_spi_transfer() / hal_spi_write_then_read() do this framing themselves.
 */

/**
 * @brief Drive CS low (select the slave).
 * @return int32_t HAL_SUCCESS, HAL_ERR_NULL_POINTER, HAL_ERR_NOT_INITIALIZED or HAL_ERR_HW_LACK.
 */
int32_t hal_spi_cs_assert(HAL_SPI *hspi);

/**
 * @brief Drive CS high (deselect the slave).
 * @return int32_t HAL_SUCCESS, HAL_ERR_NULL_POINTER, HAL_ERR_NOT_INITIALIZED or HAL_ERR_HW_LACK.
 */
int32_t hal_spi_cs_release(HAL_SPI *hspi);

/**
 * @brief Full-duplex transfer that leaves CS alone.
 *
 * Same tx/rx rules as hal_spi_transfer(); the caller owns CS.
 *
 * @return int32_t Same codes as hal_spi_transfer().
 */
int32_t hal_spi_transfer_raw(HAL_SPI *hspi, const uint8_t *tx, uint8_t *rx,
                             size_t len, size_t timeout_ms);

#if HAL_UNIT_TEST
/**
 * @brief Loopback self-test for HAL_SPI_1.
 *
 * Needs a jumper between MOSI (PA7) and MISO (PA6); no slave device attached.
 * Checks parameter validation, a full-duplex exchange, write_then_read and the
 * in-place transfer.
 *
 * @return int32_t HAL_SUCCESS if every check passed, otherwise the first error.
 */
int32_t hal_spi_unit_test(void);
#endif /* HAL_UNIT_TEST */

#ifdef __cplusplus
}
#endif

#endif /* _HAL_SPI_H_ */

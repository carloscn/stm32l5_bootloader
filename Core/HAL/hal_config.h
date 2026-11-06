#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

#define HAL_UNIT_TEST 1
#define HAL_CONFIG_RTC_DATETIME_STRING_LENGTH 12 // 12 includes 11 characters + 1 for the null terminator

/*
 * HAL_CAN_1: external MCP2518FD module on SPI1 (SCK/MISO/MOSI = PA5/PA6/PA7, CS = PD14).
 */
// Crystal on the module: 40000000u, 20000000u or 10000000u. Data rates above
// 2 Mbit/s need 40 MHz.
#define HAL_CONFIG_CAN1_MCP2518FD_OSC_HZ        40000000u
// SPI clock cap in Hz; 0 = fastest the controller allows (0.85 * OSC / 2).
#define HAL_CONFIG_CAN1_MCP2518FD_SPI_MAX_HZ    0u

#endif // HAL_CONFIG_H
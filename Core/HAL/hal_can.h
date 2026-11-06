#ifndef _HAL_CAN_H_
#define _HAL_CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "hal_config.h"

/*
 * Unified CAN / CAN FD HAL.
 *
 * Callers only see bus numbers; each bus is served by a controller backend
 * (see hal_can_backend.h):
 *
 *   HAL_CAN_1 -> external MCP2518FD on SPI1 (CS = PD14), board settings in
 *                hal_config.h (HAL_CONFIG_CAN1_MCP2518FD_*)
 *   (an on-chip FDCAN1 backend can be added later as another bus number)
 *
 * Reception is polled: hal_can_receive() takes one frame from the controller's
 * RX FIFO. hal_can_send() queues one frame; success means "queued in the
 * controller", not "ACKed on the bus".
 */

#define HAL_CAN_MAX_DATA_LEN        (64u)
#define HAL_CAN_CLASSIC_MAX_LEN     (8u)
#define HAL_CAN_STD_ID_MAX          (0x7FFu)
#define HAL_CAN_EXT_ID_MAX          (0x1FFFFFFFu)

typedef enum hal_can_num {
    HAL_CAN_1 = 0x0,
    HAL_CAN_NUM_MAX
} HAL_CAN_NUM;

/* Nominal (arbitration) / data phase bit rates. A backend rejects the rates its
 * hardware cannot time (HAL_ERR_INVALID_PARAM from hal_can_init()). For
 * classic-only traffic just use the nominal rate; the data rate is unused. */
typedef enum hal_can_bitrate {
    HAL_CAN_BITRATE_125K_500K = 0x0,
    HAL_CAN_BITRATE_250K_500K,
    HAL_CAN_BITRATE_250K_1M,
    HAL_CAN_BITRATE_250K_2M,
    HAL_CAN_BITRATE_500K_1M,
    HAL_CAN_BITRATE_500K_2M,
    HAL_CAN_BITRATE_500K_4M,
    HAL_CAN_BITRATE_500K_5M,
    HAL_CAN_BITRATE_500K_8M,
    HAL_CAN_BITRATE_1000K_4M,
    HAL_CAN_BITRATE_1000K_8M,
    HAL_CAN_BITRATE_MAX
} HAL_CAN_BITRATE;

typedef enum hal_can_mode {
    HAL_CAN_MODE_NORMAL = 0x0,          /* CAN FD + classic frames on the bus            */
    HAL_CAN_MODE_CLASSIC,               /* CAN 2.0 only (sending FD frames is rejected)  */
    HAL_CAN_MODE_LISTEN_ONLY,           /* receive only, never ACKs or sends             */
    HAL_CAN_MODE_INTERNAL_LOOPBACK,     /* TX looped back inside the controller          */
    HAL_CAN_MODE_EXTERNAL_LOOPBACK,     /* TX goes out on the bus and is received back   */
    HAL_CAN_MODE_MAX
} HAL_CAN_MODE;

typedef struct hal_can_msg_t {
    uint32_t id;            /* 11-bit (standard) or 29-bit (extended) identifier     */
    bool     extended;      /* true: 29-bit identifier                               */
    bool     fd;            /* true: CAN FD frame                                    */
    bool     brs;           /* true: switch to the data bit rate (FD frames only)    */
    uint8_t  len;           /* payload bytes: 0..8 classic; 0..8,12,16,20,24,32,48,64
                               FD. On send, an FD length between two valid sizes is
                               zero-padded up to the next size; on receive, len is
                               the size on the wire.                                 */
    uint8_t  data[HAL_CAN_MAX_DATA_LEN];
} HAL_CAN_MSG;

typedef struct hal_can_t {
    HAL_CAN_NUM     num;
    HAL_CAN_BITRATE bitrate;
    HAL_CAN_MODE    mode;
    /* Acceptance filter: a frame is accepted when (frame_id & mask) == (id & mask)
     * and its ID type matches rx_filter_extended. mask == 0 accepts every frame,
     * standard and extended (rx_filter_extended is then ignored). */
    uint32_t        rx_filter_id;
    uint32_t        rx_filter_mask;
    bool            rx_filter_extended;
} HAL_CAN;

typedef struct hal_can_error_state_t {
    uint8_t tec;            /* transmit error counter                                */
    uint8_t rec;            /* receive error counter                                 */
    bool    error_passive;  /* TX or RX error passive (counter >= 128)               */
    bool    bus_off;        /* TX bus-off (TEC > 255); controller stopped sending    */
} HAL_CAN_ERROR_STATE;

/**
 * @brief Bring up one CAN bus and switch its controller to hcan->mode.
 *
 * @return int32_t HAL_SUCCESS,
 *                 HAL_ERR_NULL_POINTER / HAL_ERR_INVALID_PARAM (bad config or a
 *                 bit rate the controller cannot time),
 *                 HAL_ERR_HW_LACK (bus not available on this board),
 *                 HAL_ERR_COMMUNICATION (controller does not answer),
 *                 HAL_ERR_TIMEOUT (controller clock never became ready),
 *                 HAL_ERR_HW_STATE (mode switch did not complete, e.g. no
 *                 transceiver/bus in NORMAL mode),
 *                 HAL_ERR_INIT (controller configuration failed).
 */
int32_t hal_can_init(HAL_CAN *hcan);

/**
 * @brief Stop the controller (configuration mode) and release its resources.
 */
void hal_can_deinit(HAL_CAN *hcan);

/**
 * @brief Queue one frame for transmission, waiting up to timeout_ms for room.
 *
 * @return int32_t HAL_SUCCESS, HAL_ERR_NULL_POINTER, HAL_ERR_NOT_INITIALIZED,
 *                 HAL_ERR_INVALID_PARAM (id/len/flags, or FD in CLASSIC mode,
 *                 or any send in LISTEN_ONLY mode), HAL_ERR_TIMEOUT (TX FIFO
 *                 stayed full), HAL_ERR_HW_SEND.
 */
int32_t hal_can_send(HAL_CAN *hcan, const HAL_CAN_MSG *msg, size_t timeout_ms);

/**
 * @brief Take one received frame, waiting up to timeout_ms (0: check once).
 *
 * @return int32_t HAL_SUCCESS, HAL_ERR_NULL_POINTER, HAL_ERR_NOT_INITIALIZED,
 *                 HAL_ERR_TIMEOUT (nothing received), HAL_ERR_HW_RECV.
 */
int32_t hal_can_receive(HAL_CAN *hcan, HAL_CAN_MSG *msg, size_t timeout_ms);

/**
 * @brief Read the bus error counters and state.
 *
 * @return int32_t HAL_SUCCESS, HAL_ERR_NULL_POINTER, HAL_ERR_NOT_INITIALIZED, HAL_ERR_HW_READ.
 */
int32_t hal_can_get_error_state(HAL_CAN *hcan, HAL_CAN_ERROR_STATE *state);

/**
 * @brief Payload size a frame of len bytes occupies on the wire.
 *
 * Classic: len (0..8). FD: the next valid FD size (e.g. 10 -> 12).
 * Returns 0 for an invalid length (classic > 8, FD > 64).
 */
uint8_t hal_can_wire_len(uint8_t len, bool fd);

#if HAL_UNIT_TEST
/**
 * @brief Self-test of one bus in internal loopback mode.
 *
 * Needs only the controller (no CAN bus, no other node). Sends a classic
 * standard frame, an FD extended frame with BRS and a padded FD frame, checks
 * that each comes back unchanged, then checks parameter validation.
 *
 * @return int32_t HAL_SUCCESS if every check passed, otherwise the first error.
 */
int32_t hal_can_unit_test(HAL_CAN_NUM num);
#endif /* HAL_UNIT_TEST */

#ifdef __cplusplus
}
#endif

#endif /* _HAL_CAN_H_ */

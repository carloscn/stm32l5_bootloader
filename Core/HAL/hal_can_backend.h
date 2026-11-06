#ifndef _HAL_CAN_BACKEND_H_
#define _HAL_CAN_BACKEND_H_

/*
 * Controller backend interface for hal_can. Internal: only hal_can.c and the
 * backends include this.
 *
 * hal_can.c validates everything that does not depend on the controller
 * before calling a backend:
 *   - init:    num/bitrate/mode in range, filter id fits its ID type
 *   - send:    bus ready, id range, len (classic <= 8, FD <= 64),
 *              brs only with fd, no FD in CLASSIC mode, no send in LISTEN_ONLY
 *   - receive / error_state: bus ready, non-NULL out pointers
 * A backend only has to do the hardware work and report hal_err.h codes.
 *
 * Message lengths: hal_can.c hands send() a message whose payload is already
 * zero-padded to the wire length (hal_can_wire_len()), with msg->len set to
 * that wire length. receive() must report the wire length as well.
 *
 * Each backend drives exactly one controller instance.
 */

#include <stdint.h>
#include <stddef.h>
#include "hal_can.h"

typedef struct hal_can_backend_t {
    const char *name;
    int32_t (*init)(const HAL_CAN *hcan);
    void    (*deinit)(void);
    int32_t (*send)(const HAL_CAN_MSG *msg, size_t timeout_ms);
    int32_t (*receive)(HAL_CAN_MSG *msg, size_t timeout_ms);
    int32_t (*get_error_state)(HAL_CAN_ERROR_STATE *state);
} HAL_CAN_BACKEND;

/* External MCP2518FD on SPI (Core/Drivers/mcp2518fd/can_mcp2518fd.c). */
extern const HAL_CAN_BACKEND hal_can_backend_mcp2518fd;

#endif /* _HAL_CAN_BACKEND_H_ */

/*
 * Unified CAN / CAN FD HAL: parameter checks shared by every controller, then
 * dispatch to the backend that serves the bus (see hal_can_backend.h).
 */
#include <string.h>
#include "hal_can.h"
#include "hal_can_backend.h"
#include "hal_err.h"

/* Bus number -> controller backend on this board. */
static const HAL_CAN_BACKEND *const can_backends[HAL_CAN_NUM_MAX] = {
    [HAL_CAN_1] = &hal_can_backend_mcp2518fd,
};

static bool can_ready[HAL_CAN_NUM_MAX];
static HAL_CAN_MODE can_mode[HAL_CAN_NUM_MAX];

/* FD payload sizes for DLC 9..15 (DLC 0..8 map 1:1). */
static const uint8_t can_fd_sizes[] = { 12, 16, 20, 24, 32, 48, 64 };

uint8_t hal_can_wire_len(uint8_t len, bool fd)
{
    uint8_t i = 0;

    if (len <= HAL_CAN_CLASSIC_MAX_LEN) {
        return len;
    }
    if (!fd) {
        return 0;
    }
    for (i = 0; i < sizeof(can_fd_sizes); i++) {
        if (len <= can_fd_sizes[i]) {
            return can_fd_sizes[i];
        }
    }
    return 0;
}

static bool can_id_valid(uint32_t id, bool extended)
{
    return id <= (extended ? HAL_CAN_EXT_ID_MAX : HAL_CAN_STD_ID_MAX);
}

/* Backend of an initialized bus, or NULL with *err set. */
static const HAL_CAN_BACKEND *can_ready_backend(const HAL_CAN *hcan, int32_t *err)
{
    if (hcan == NULL) {
        *err = HAL_ERR_NULL_POINTER;
        return NULL;
    }
    if (hcan->num >= HAL_CAN_NUM_MAX || can_backends[hcan->num] == NULL) {
        *err = HAL_ERR_HW_LACK;
        return NULL;
    }
    if (!can_ready[hcan->num]) {
        *err = HAL_ERR_NOT_INITIALIZED;
        return NULL;
    }
    *err = HAL_SUCCESS;
    return can_backends[hcan->num];
}

int32_t hal_can_init(HAL_CAN *hcan)
{
    int32_t ret = HAL_SUCCESS;
    const HAL_CAN_BACKEND *be = NULL;

    if (hcan == NULL) {
        return HAL_ERR_NULL_POINTER;
    }
    if (hcan->num >= HAL_CAN_NUM_MAX || can_backends[hcan->num] == NULL) {
        return HAL_ERR_HW_LACK;
    }
    if (hcan->bitrate >= HAL_CAN_BITRATE_MAX || hcan->mode >= HAL_CAN_MODE_MAX ||
        !can_id_valid(hcan->rx_filter_id, hcan->rx_filter_extended) ||
        !can_id_valid(hcan->rx_filter_mask, hcan->rx_filter_extended)) {
        return HAL_ERR_INVALID_PARAM;
    }

    be = can_backends[hcan->num];
    if (can_ready[hcan->num]) {
        be->deinit();                   /* re-init with the new settings */
        can_ready[hcan->num] = false;
    }

    ret = be->init(hcan);
    if (ret == HAL_SUCCESS) {
        can_ready[hcan->num] = true;
        can_mode[hcan->num] = hcan->mode;
    }
    return ret;
}

void hal_can_deinit(HAL_CAN *hcan)
{
    if (hcan == NULL || hcan->num >= HAL_CAN_NUM_MAX || can_backends[hcan->num] == NULL) {
        return;
    }
    if (can_ready[hcan->num]) {
        can_backends[hcan->num]->deinit();
        can_ready[hcan->num] = false;
    }
}

int32_t hal_can_send(HAL_CAN *hcan, const HAL_CAN_MSG *msg, size_t timeout_ms)
{
    int32_t ret = HAL_SUCCESS;
    const HAL_CAN_BACKEND *be = can_ready_backend(hcan, &ret);
    HAL_CAN_MSG wire;
    uint8_t wire_len = 0;

    if (be == NULL) {
        return ret;
    }
    if (msg == NULL) {
        return HAL_ERR_NULL_POINTER;
    }

    wire_len = hal_can_wire_len(msg->len, msg->fd);
    if (!can_id_valid(msg->id, msg->extended) ||
        (msg->len != 0u && wire_len == 0u) ||
        (msg->brs && !msg->fd) ||
        (msg->fd && can_mode[hcan->num] == HAL_CAN_MODE_CLASSIC) ||
        can_mode[hcan->num] == HAL_CAN_MODE_LISTEN_ONLY) {
        return HAL_ERR_INVALID_PARAM;
    }

    /* Hand the backend a frame already padded to its wire length. */
    wire.id = msg->id;
    wire.extended = msg->extended;
    wire.fd = msg->fd;
    wire.brs = msg->brs;
    wire.len = wire_len;
    memcpy(wire.data, msg->data, msg->len);
    memset(&wire.data[msg->len], 0, (size_t)(wire_len - msg->len));

    return be->send(&wire, timeout_ms);
}

int32_t hal_can_receive(HAL_CAN *hcan, HAL_CAN_MSG *msg, size_t timeout_ms)
{
    int32_t ret = HAL_SUCCESS;
    const HAL_CAN_BACKEND *be = can_ready_backend(hcan, &ret);

    if (be == NULL) {
        return ret;
    }
    if (msg == NULL) {
        return HAL_ERR_NULL_POINTER;
    }
    return be->receive(msg, timeout_ms);
}

int32_t hal_can_get_error_state(HAL_CAN *hcan, HAL_CAN_ERROR_STATE *state)
{
    int32_t ret = HAL_SUCCESS;
    const HAL_CAN_BACKEND *be = can_ready_backend(hcan, &ret);

    if (be == NULL) {
        return ret;
    }
    if (state == NULL) {
        return HAL_ERR_NULL_POINTER;
    }
    memset(state, 0, sizeof(*state));
    return be->get_error_state(state);
}

#if HAL_UNIT_TEST
static int32_t can_loopback_one(HAL_CAN *hcan, const HAL_CAN_MSG *tx)
{
    HAL_CAN_MSG rx;
    int32_t ret = HAL_SUCCESS;
    uint8_t expect_len = hal_can_wire_len(tx->len, tx->fd);

    ret = hal_can_send(hcan, tx, 10);
    if (ret != HAL_SUCCESS) {
        return ret;
    }
    memset(&rx, 0x5A, sizeof(rx));
    ret = hal_can_receive(hcan, &rx, 20);
    if (ret != HAL_SUCCESS) {
        return ret;
    }
    if (rx.id != tx->id || rx.extended != tx->extended || rx.fd != tx->fd ||
        rx.brs != tx->brs || rx.len != expect_len ||
        memcmp(rx.data, tx->data, tx->len) != 0) {
        return HAL_ERR_COMMUNICATION;
    }
    for (uint8_t i = tx->len; i < expect_len; i++) {
        if (rx.data[i] != 0u) {         /* padding must be zero */
            return HAL_ERR_COMMUNICATION;
        }
    }
    return HAL_SUCCESS;
}

int32_t hal_can_unit_test(HAL_CAN_NUM num)
{
    int32_t ret = HAL_SUCCESS;
    HAL_CAN can = {
        .num = num,
        .bitrate = HAL_CAN_BITRATE_500K_2M,
        .mode = HAL_CAN_MODE_INTERNAL_LOOPBACK,
        .rx_filter_mask = 0,                    /* accept everything */
    };
    HAL_CAN_MSG msg;
    uint8_t i = 0;

    /* Checks that need no hardware. */
    if (hal_can_init(NULL) != HAL_ERR_NULL_POINTER ||
        hal_can_wire_len(9, false) != 0u || hal_can_wire_len(9, true) != 12u ||
        hal_can_wire_len(64, true) != 64u || hal_can_wire_len(65, true) != 0u) {
        return HAL_ERR;
    }
    memset(&msg, 0, sizeof(msg));
    if (hal_can_send(&can, &msg, 0) != HAL_ERR_NOT_INITIALIZED) {
        return HAL_ERR;
    }

    ret = hal_can_init(&can);
    if (ret != HAL_SUCCESS) {
        return ret;
    }

    /* 1. Classic, standard ID, 3 bytes. */
    memset(&msg, 0, sizeof(msg));
    msg.id = 0x123;
    msg.len = 3;
    msg.data[0] = 0xDE; msg.data[1] = 0xAD; msg.data[2] = 0xBE;
    ret = can_loopback_one(&can, &msg);
    if (ret != HAL_SUCCESS) {
        goto out;
    }

    /* 2. FD, extended ID, BRS, 64 bytes. */
    memset(&msg, 0, sizeof(msg));
    msg.id = 0x18DA00F1u;
    msg.extended = true;
    msg.fd = true;
    msg.brs = true;
    msg.len = 64;
    for (i = 0; i < 64u; i++) {
        msg.data[i] = (uint8_t)(i * 7u + 1u);
    }
    ret = can_loopback_one(&can, &msg);
    if (ret != HAL_SUCCESS) {
        goto out;
    }

    /* 3. FD, 10 bytes -> 12 on the wire, 2 zero padding bytes. */
    memset(&msg, 0, sizeof(msg));
    msg.id = 0x7FF;
    msg.fd = true;
    msg.len = 10;
    memset(msg.data, 0xA5, msg.len);
    ret = can_loopback_one(&can, &msg);
    if (ret != HAL_SUCCESS) {
        goto out;
    }

    /* 4. RX FIFO is empty again. */
    if (hal_can_receive(&can, &msg, 0) != HAL_ERR_TIMEOUT) {
        ret = HAL_ERR;
        goto out;
    }

    /* 5. Rejected frames: bad std ID, BRS without FD, classic > 8 bytes. */
    memset(&msg, 0, sizeof(msg));
    msg.id = 0x800;
    if (hal_can_send(&can, &msg, 0) != HAL_ERR_INVALID_PARAM) {
        ret = HAL_ERR;
        goto out;
    }
    msg.id = 0x100;
    msg.brs = true;
    if (hal_can_send(&can, &msg, 0) != HAL_ERR_INVALID_PARAM) {
        ret = HAL_ERR;
        goto out;
    }
    msg.brs = false;
    msg.len = 9;
    if (hal_can_send(&can, &msg, 0) != HAL_ERR_INVALID_PARAM) {
        ret = HAL_ERR;
        goto out;
    }

out:
    hal_can_deinit(&can);
    return ret;
}
#endif /* HAL_UNIT_TEST */

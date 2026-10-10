#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>

#include "dt7_remote_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DJI DBUS/DT7 sends five channels, two switches, mouse and keyboard data. */
#define RC_DBUS_FRAME_LENGTH       18U
#define RC_DBUS_CHANNEL_MIN        DT7_RC_CH_VALUE_MIN
#define RC_DBUS_CHANNEL_MAX        DT7_RC_CH_VALUE_MAX
#define RC_DBUS_ONLINE_TIMEOUT_MS  100U

typedef struct {
    DT7RecData raw;
    DT7CmdData command;

    uint32_t frame_count;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t error_count;
    uint32_t last_frame_tick;

    uint8_t online;
    uint8_t frame_ready;
    uint8_t stream_size;
    uint8_t stream[RC_DBUS_FRAME_LENGTH];
} RC_Remote_t;

/* Global instance used by ReceiveTask and application tasks. */
extern RC_Remote_t rc_remote;

void RC_Remote_Init(RC_Remote_t *remote);

/* Feed one packet popped from BSP_UART.  The input may contain partial or
   multiple DBUS frames; the parser keeps state across calls. */
void RC_Remote_Input(RC_Remote_t *remote, const uint8_t *data,
                     uint16_t size);

/* Drop an incomplete frame after a UART/DMA error so the next frame can
   re-synchronise from a clean boundary. */
void RC_Remote_OnError(RC_Remote_t *remote);

/* Copy the most recently decoded values.  Returns zero before the first valid
   frame and one after at least one valid frame has been decoded. */
uint8_t RC_Remote_Read(const RC_Remote_t *remote, DT7RecData *raw,
                       DT7CmdData *command);

/* Check freshness using a caller supplied RTOS tick converted to ms. */
uint8_t RC_Remote_IsOnline(const RC_Remote_t *remote, uint32_t now_ms,
                           uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* REMOTE_CONTROL_H */

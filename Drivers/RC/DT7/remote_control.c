#include "remote_control.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static uint16_t RC_Remote_Channel(const uint8_t *frame, uint8_t channel)
{
    switch (channel) {
    case 0U:
        return (uint16_t)((frame[0] | ((uint16_t)frame[1] << 8U)) & 0x07FFU);
    case 1U:
        return (uint16_t)((((uint16_t)frame[1] >> 3U) |
                           ((uint16_t)frame[2] << 5U)) & 0x07FFU);
    case 2U:
        return (uint16_t)((((uint16_t)frame[2] >> 6U) |
                           ((uint16_t)frame[3] << 2U) |
                           ((uint16_t)frame[4] << 10U)) & 0x07FFU);
    case 3U:
        return (uint16_t)((((uint16_t)frame[4] >> 1U) |
                           ((uint16_t)frame[5] << 7U)) & 0x07FFU);
    default:
        return (uint16_t)((frame[16] | ((uint16_t)frame[17] << 8U)) &
                          0x07FFU);
    }
}

static uint8_t RC_Remote_FrameValid(const uint8_t *frame)
{
    uint8_t channel;
    uint8_t switch_left;
    uint8_t switch_right;

    if (frame == NULL) {
        return 0U;
    }

    /* The two switches occupy adjacent two-bit fields in byte 5. */
    switch_left = (uint8_t)((frame[5] >> 6U) & 0x03U);
    switch_right = (uint8_t)((frame[5] >> 4U) & 0x03U);
    if (switch_left == 0U || switch_left > 3U ||
        switch_right == 0U || switch_right > 3U) {
        return 0U;
    }

    /* Byte 17 contains only the upper three bits of channel 4. */
    if ((frame[17] & 0xF8U) != 0U) {
        return 0U;
    }

    for (channel = 0U; channel < 5U; ++channel) {
        uint16_t value = RC_Remote_Channel(frame, channel);
        if (value < RC_DBUS_CHANNEL_MIN || value > RC_DBUS_CHANNEL_MAX) {
            return 0U;
        }
    }
    return 1U;
}

static void RC_Remote_Decode(RC_Remote_t *remote,
                             const uint8_t *frame)
{
    DT7RecData raw;
    DT7CmdData command;

    DT7RawDataUpdate(&raw, frame);
    DT7DataProcess(&command, &raw);

    taskENTER_CRITICAL();
    remote->raw = raw;
    remote->command = command;
    remote->frame_count++;
    remote->valid_frame_count++;
    remote->frame_ready = 1U;
    remote->online = 1U;
    remote->last_frame_tick = (uint32_t)xTaskGetTickCount();
    taskEXIT_CRITICAL();
}

void RC_Init(RC_Remote_t *remote)
{
    if (remote == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    memset(remote, 0, sizeof(*remote));
    remote->raw.ch0 = DT7_RC_CH_VALUE_OFFSET;
    remote->raw.ch1 = DT7_RC_CH_VALUE_OFFSET;
    remote->raw.ch2 = DT7_RC_CH_VALUE_OFFSET;
    remote->raw.ch3 = DT7_RC_CH_VALUE_OFFSET;
    remote->raw.ch4 = DT7_RC_CH_VALUE_OFFSET;
    remote->raw.switch_left = DT7_RC_SW_UP;
    remote->raw.switch_right = DT7_RC_SW_UP;
    remote->command.ch0 = 0;
    remote->command.ch1 = 0;
    remote->command.ch2 = 0;
    remote->command.ch3 = 0;
    remote->command.ch4 = 0;
    remote->command.switch_left = DT7_RC_SW_UP;
    remote->command.switch_right = DT7_RC_SW_UP;
    taskEXIT_CRITICAL();
}

void RC_Remote_Input(RC_Remote_t *remote, const uint8_t *data,
                     uint16_t size)
{
    uint16_t i;

    if (remote == NULL || data == NULL || size == 0U) {
        return;
    }

    for (i = 0U; i < size; ++i) {
        if (remote->stream_size < RC_DBUS_FRAME_LENGTH) {
            remote->stream[remote->stream_size++] = data[i];
        }

        if (remote->stream_size == RC_DBUS_FRAME_LENGTH) {
            if (RC_Remote_FrameValid(remote->stream) != 0U) {
                RC_Remote_Decode(remote, remote->stream);
                remote->stream_size = 0U;
            } else {
                /* Shift one byte and keep searching.  This handles a packet
                   boundary in the middle of a frame and random UART noise. */
                memmove(remote->stream, remote->stream + 1U,
                        RC_DBUS_FRAME_LENGTH - 1U);
                remote->stream_size = RC_DBUS_FRAME_LENGTH - 1U;
                remote->invalid_frame_count++;
            }
        }
    }
}

void RC_Remote_OnError(RC_Remote_t *remote)
{
    if (remote == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    remote->stream_size = 0U;
    remote->error_count++;
    remote->online = 0U;
    taskEXIT_CRITICAL();
}

uint8_t RC_Remote_Read(const RC_Remote_t *remote, DT7RecData *raw,
                       DT7CmdData *command)
{
    uint8_t ready;

    if (remote == NULL || raw == NULL || command == NULL) {
        return 0U;
    }

    taskENTER_CRITICAL();
    *raw = remote->raw;
    *command = remote->command;
    ready = remote->frame_ready;
    taskEXIT_CRITICAL();
    return ready;
}

uint8_t RC_Remote_IsOnline(const RC_Remote_t *remote, uint32_t now_ms,
                           uint32_t timeout_ms)
{
    uint32_t last_tick;
    uint8_t online;
    uint32_t timeout_ticks;

    if (remote == NULL) {
        return 0U;
    }

    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    taskENTER_CRITICAL();
    last_tick = remote->last_frame_tick;
    online = remote->online;
    taskEXIT_CRITICAL();

    /* now_ms is retained in the API for callers that use millisecond time;
       the FreeRTOS tick is the authoritative timestamp in this task model. */
    (void)now_ms;
    if (online == 0U || timeout_ticks == 0U) {
        return online;
    }
    return ((uint32_t)xTaskGetTickCount() - last_tick) <= timeout_ticks;
}

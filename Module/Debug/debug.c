#include "debug.h"

#include <string.h>

#include "vofa_uart.h"

void Debug_Init(Debug_t *debug, const DebugConfig_t *config)
{
    memset(debug, 0, sizeof(*debug));
    debug->config = *config;
    if (debug->config.param.channel_count > DEBUG_CHANNEL_MAX)
    {
        debug->config.param.channel_count = DEBUG_CHANNEL_MAX;
    }
    /* 帧缓冲就在 debug 里，这里在最终地址上绑定，不能先在栈上建好再拷 */
    (void)VofaJustFloat_Init(&debug->frame, debug->frame_buffer, sizeof(debug->frame_buffer));
}

BaseType_t Debug_Publish(QueueHandle_t queue, uint8_t first_channel,
                         const float *value, uint8_t count)
{
    DebugMsg_t msg = {0};

    if (queue == NULL || value == NULL || count == 0U || count > DEBUG_MSG_FLOATS_MAX ||
        (uint16_t)first_channel + count > DEBUG_CHANNEL_MAX)
    {
        return pdFAIL;
    }

    msg.first_channel = first_channel;
    msg.count = count;
    memcpy(msg.value, value, (size_t)count * sizeof(float));
    return xQueueSend(queue, &msg, 0U);
}

/* 消息可能带着超出本帧通道数的通道，超出部分只存进数组，不发 */
static void Debug_Apply(Debug_t *debug, const DebugMsg_t *msg)
{
    for (uint8_t i = 0U; i < msg->count; ++i)
    {
        uint16_t index = (uint16_t)msg->first_channel + i;
        if (index < DEBUG_CHANNEL_MAX)
        {
            debug->channel[index] = msg->value[i];
        }
    }
}

void Debug_Transmit(Debug_t *debug, UartInstance_t *uart)
{
    DebugMsg_t msg;

    while (debug->config.comm.msg_queue != NULL &&
           xQueueReceive(debug->config.comm.msg_queue, &msg, 0U) == pdPASS)
    {
        Debug_Apply(debug, &msg);
    }

    VofaJustFloat_Reset(&debug->frame);
    if (!VofaJustFloat_AddFloats(&debug->frame, debug->channel, debug->config.param.channel_count))
    {
        return;
    }
    /* 忙就丢这一帧，返回值不用处理 */
    (void)VofaJustFloat_Send(&debug->frame, uart);
}

static void Debug_OnUartTransmit(void *context, UartInstance_t *uart)
{
    Debug_Transmit((Debug_t *)context, uart);
}

BaseType_t Debug_BindUart(Debug_t *debug, UartInstance_t *uart)
{
    return UART_BindTx(uart, Debug_OnUartTransmit, debug);
}

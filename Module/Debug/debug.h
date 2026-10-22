#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "uart_driver.h"
#include "vofa_justfloat.h"

/* 调试数据输出：各模块把要看的数据发到队列，UART 发送任务调 Debug_Transmit 整理成 VOFA JustFloat 帧发出去
 * 通道是一个固定大小的 float 数组，每个模块占其中连续的几个通道，通道号由 App 层统一分配（见 system_define.h）
 * 数据只保留最新值：队列里的消息按先后写进通道数组，没更新的通道保持上次的值
 * 队列满了丢新消息，发布方不阻塞，不影响控制周期 */
#define DEBUG_CHANNEL_MAX        16U    /* 通道数组大小，一帧最多这么多通道 */
#define DEBUG_MSG_FLOATS_MAX     6U     /* 一条消息最多带多少个 float */

/*--------------------------------------输入层--------------------------------------*/
/* 队列里的元素：从 first_channel 起连续 count 个通道的新值 */
typedef struct {
    uint8_t first_channel;
    uint8_t count;
    float value[DEBUG_MSG_FLOATS_MAX];
} DebugMsg_t;

/*--------------------------------------通信层--------------------------------------*/
typedef struct {
    QueueHandle_t msg_queue;    /* 元素 DebugMsg_t，由 System 层创建 */
} DebugComm_t;

/*--------------------------------------参数层--------------------------------------*/
typedef struct {
    uint8_t channel_count;      /* 每帧发多少个通道，从通道 0 起，不超过 DEBUG_CHANNEL_MAX */
} DebugParam_t;

typedef struct {
    DebugComm_t comm;
    DebugParam_t param;
} DebugConfig_t;

typedef struct {
    DebugConfig_t config;
    float channel[DEBUG_CHANNEL_MAX];
    VofaJustFloat_t frame;
    uint8_t frame_buffer[DEBUG_CHANNEL_MAX * VOFA_JUSTFLOAT_CHANNEL_SIZE + VOFA_JUSTFLOAT_TAIL_SIZE];
} Debug_t;

void Debug_Init(Debug_t *debug, const DebugConfig_t *config);

/* 把调试器挂到串口的发送口上，之后发送任务每个周期经 UART_DispatchTx 调 Debug_Transmit
 * 在 Debug_Init 之后调用，用法同 RC_Remote_BindUart */
BaseType_t Debug_BindUart(Debug_t *debug, UartInstance_t *uart);

/* 发布方调用：把 count 个值写到从 first_channel 起的通道，队列满或参数无效返回 pdFAIL，不阻塞
 * 只能在任务里调，不能在中断里调 */
BaseType_t Debug_Publish(QueueHandle_t queue, uint8_t first_channel,
                         const float *value, uint8_t count);

/* 发送任务调用：取空队列更新通道数组，拼帧，经 uart 发出
 * 上一帧还没发完（HAL_BUSY）就丢掉这一帧，下个周期会带着更新的值再发 */
void Debug_Transmit(Debug_t *debug, UartInstance_t *uart);

#endif

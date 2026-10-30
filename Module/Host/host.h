#ifndef HOST_H
#define HOST_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "arm.h"
#include "uart_driver.h"

/* 上位机通信模块：把七轴臂的观测与状态按串口协议（Host/docs/02_protocol.md）打成 JOINT_STATE 帧，经一个 UART 发给上位机
 * 目前只做发送：帧格式、字段偏移、CRC 与上位机 Host/armhost/protocol 逐字节一致，改动要同步两边
 * 发送回调由 UART 发送任务按周期调用，直接读传入的 Arm_t（不走队列，只读不写）；Arm 任务正在写时可能读到相邻两周期混合的一帧，上报用途可接受
 * 上一帧还在发（HAL_BUSY）就丢掉这一帧，下个上报周期带新数据再发，序号照常递增，上位机能据此统计丢帧
 * status_flags 里的遥控在线位模块不填（0），需要时由 System 层在拿到帧之前自行扩展
 */
#define HOST_FRAME_SOF0             0xA5U
#define HOST_FRAME_SOF1             0x5AU
#define HOST_FRAME_VERSION          0x10U   /* 高 4 位 major，低 4 位 minor */
#define HOST_FRAME_HEADER_SIZE      9U      /* 含帧头 CRC-8 */
#define HOST_FRAME_TRAILER_SIZE     2U      /* CRC-16 */
#define HOST_FRAME_OVERHEAD         (HOST_FRAME_HEADER_SIZE + HOST_FRAME_TRAILER_SIZE)
#define HOST_FRAME_MAX_SIZE         UART_TX_BUFFER_SIZE     /* 受 UART 发送缓冲限制，协议规定最大 256 */
#define HOST_MSG_JOINT_STATE        0x10U

#define HOST_JOINT_RECORD_SIZE      28U     /* 每个关节一条记录，v1.0 */
#define HOST_JOINT_STATE_HEAD_SIZE  12U
#define HOST_JOINT_STATE_SIZE       (HOST_JOINT_STATE_HEAD_SIZE + ARM_JOINT_COUNT * HOST_JOINT_RECORD_SIZE)
#define HOST_JOINT_STATE_FRAME_SIZE (HOST_FRAME_OVERHEAD + HOST_JOINT_STATE_SIZE)

/* status_flags */
#define HOST_FLAG_CTRL_ALIGNING     0x0010U
/* joint_flags */
#define HOST_JOINT_FLAG_ONLINE      0x01U

/*--------------------------------------外设层--------------------------------------*/
typedef struct {
    UartInstance_t *uart;       /* 连上位机的串口，发送口绑在这个实例上 */
} HostPeriph_t;

/*--------------------------------------参数层--------------------------------------*/
typedef struct {
    uint16_t state_divider;     /* 每调用多少次发送回调上报一帧，发送任务周期 2 ms 时取 5 即 100 Hz，取 0 按 1 处理 */
} HostParam_t;

typedef struct {
    HostPeriph_t periph;
    HostParam_t param;
} HostConfig_t;

typedef struct {
    HostConfig_t config;
    const Arm_t *arm;           /* 只读，由 Host_Init 传入 */
    uint16_t divider_count;
    uint16_t sequence;          /* 发送方每帧 +1，回绕 */
    uint32_t drop_count;        /* 因 DMA 忙丢掉的上报帧 */
    uint8_t frame[HOST_FRAME_MAX_SIZE];
} Host_t;

void Host_Init(Host_t *host, const HostConfig_t *config, const Arm_t *arm);

/* 把上位机挂到配置里那个串口的发送口上，之后发送任务每个周期经 UART_DispatchTx 调 Host_Transmit
 * 在 Host_Init 之后调用，用法同 Debug_BindUart */
BaseType_t Host_BindUart(Host_t *host);

/* 发送任务调用：到上报周期就拼一帧 JOINT_STATE 发出，没到周期直接返回；上一帧没发完就丢掉这一帧 */
void Host_Transmit(Host_t *host, UartInstance_t *uart);

/* 校验算法放出来给单元测试和以后的接收解析复用：CRC-8/SMBUS 与 CRC-16/CCITT-FALSE */
uint8_t Host_Crc8(const uint8_t *data, uint16_t length);
uint16_t Host_Crc16(const uint8_t *data, uint16_t length);

#endif

#ifndef HOST_H
#define HOST_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "arm.h"
#include "uart_driver.h"

/* Host protocol bridge: periodic JOINT_STATE reports plus commands from the PC.
 * UART RX runs in interrupt context and only validates/queues frames. Command
 * execution, Arm writes and all responses run from Host_Transmit in task context.
 */
#define HOST_FRAME_SOF0             0xA5U
#define HOST_FRAME_SOF1             0x5AU
#define HOST_FRAME_VERSION          0x10U
#define HOST_FRAME_HEADER_SIZE      9U
#define HOST_FRAME_TRAILER_SIZE     2U
#define HOST_FRAME_OVERHEAD         (HOST_FRAME_HEADER_SIZE + HOST_FRAME_TRAILER_SIZE)
#define HOST_FRAME_MAX_SIZE         UART_TX_BUFFER_SIZE
#define HOST_FRAME_MAX_PAYLOAD      (HOST_FRAME_MAX_SIZE - HOST_FRAME_OVERHEAD)

#define HOST_MSG_HEARTBEAT          0x01U
#define HOST_MSG_DEVICE_INFO_REQ    0x02U
#define HOST_MSG_DEVICE_INFO        0x03U
#define HOST_MSG_ACK                0x04U
#define HOST_MSG_JOINT_STATE        0x10U
#define HOST_MSG_SET_HOST_TARGET    0x23U
#define HOST_MSG_PARAM_READ         0x28U
#define HOST_MSG_PARAM_VALUE        0x2AU

#define HOST_JOINT_RECORD_SIZE      28U
#define HOST_JOINT_STATE_HEAD_SIZE  12U
#define HOST_JOINT_STATE_SIZE       (HOST_JOINT_STATE_HEAD_SIZE + ARM_JOINT_COUNT * HOST_JOINT_RECORD_SIZE)
#define HOST_JOINT_STATE_FRAME_SIZE (HOST_FRAME_OVERHEAD + HOST_JOINT_STATE_SIZE)

#define HOST_ACK_SIZE               8U
#define HOST_PARAM_VALUE_SIZE       12U
#define HOST_DEVICE_INFO_SIZE       28U
#define HOST_SET_HOST_TARGET_SIZE   8U
#define HOST_TARGET_ACTION_SET_OPEN    0U  /* 保存目标，该关节不闭环 */
#define HOST_TARGET_ACTION_SET_CLOSED  1U  /* 保存目标，该关节投入闭环 */
#define HOST_TARGET_ACTION_STOP_ALL    2U  /* 全部关节退出闭环，目标保留 */

#define HOST_PARAM_STATE_RATE_HZ    0x0001U
#define HOST_PARAM_STATUS_RATE_HZ   0x0002U
#define HOST_PARAM_LINK_TIMEOUT     0x0003U
#define HOST_PARAM_TORQUE_HOLD_MAX  0x0004U
#define HOST_PARAM_ZERO_POINT       0x0100U
#define HOST_PARAM_DIRECTION        0x0101U
#define HOST_PARAM_LIMIT_MIN        0x0102U
#define HOST_PARAM_LIMIT_MAX        0x0103U
#define HOST_PARAM_GAIN_KP          0x0200U
#define HOST_PARAM_GAIN_KI          0x0201U
#define HOST_PARAM_GAIN_KD          0x0202U
#define HOST_PARAM_GAIN_D_ALPHA     0x0203U
#define HOST_PARAM_GAIN_SUM_MAX     0x0204U
#define HOST_PARAM_GAIN_OUTPUT_MAX  0x0205U
#define HOST_PARAM_HOST_TORQUE_LIMIT 0x0300U
#define HOST_PARAM_DH               0x0400U

/* status_flags */
#define HOST_FLAG_RC_ONLINE         0x0001U
#define HOST_FLAG_HOST_LINK_OK      0x0002U
#define HOST_FLAG_CTRL_ALIGNING     0x0010U
#define HOST_FLAG_HOST_CLOSED_LOOP  0x0020U
/* joint_flags */
#define HOST_JOINT_FLAG_ONLINE      0x01U
#define HOST_JOINT_FLAG_HOST_TARGET_VALID 0x10U
#define HOST_JOINT_FLAG_HOST_CLOSED_LOOP  0x20U

#define HOST_LINK_TIMEOUT_MS        300U
#define HOST_CMD_AGE_NONE           0xFFFFU
#define HOST_RX_BUFFER_SIZE         HOST_FRAME_MAX_SIZE
#define HOST_MAX_REQUEST_PAYLOAD    32U
#define HOST_RX_QUEUE_SIZE          16U
#define HOST_TX_QUEUE_SIZE          8U
#define HOST_REPLY_CACHE_SIZE       8U

typedef struct {
    UartInstance_t *uart;
} HostPeriph_t;

typedef struct {
    uint16_t state_divider;
} HostParam_t;

typedef struct {
    HostPeriph_t periph;
    HostParam_t param;
} HostConfig_t;

typedef struct {
    uint16_t seq;
    uint8_t type;
    uint8_t flags;
    uint8_t length;
    uint8_t overflow;
    uint8_t payload[HOST_MAX_REQUEST_PAYLOAD];
} HostRequest_t;

typedef struct {
    uint8_t type;
    uint8_t length;
    uint8_t payload[HOST_DEVICE_INFO_SIZE];
} HostResponse_t;

typedef struct {
    uint16_t seq;
    uint8_t type;
    uint8_t length;
    uint8_t payload[HOST_DEVICE_INFO_SIZE];
} HostReplyCache_t;

typedef struct {
    HostConfig_t config;
    Arm_t *arm;
    uint16_t divider_count;
    uint16_t sequence;
    uint32_t drop_count;
    uint32_t rx_drop_count;
    uint32_t tx_drop_count;
    uint32_t last_rx_ms;
    uint32_t last_cmd_ms;
    uint8_t ever_rx;
    uint8_t ever_cmd;
    uint8_t rx_buffer[HOST_RX_BUFFER_SIZE];
    uint16_t rx_length;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    uint16_t tx_head;
    uint16_t tx_tail;
    uint16_t cache_head;
    uint8_t cache_count;
    HostRequest_t requests[HOST_RX_QUEUE_SIZE];
    HostResponse_t responses[HOST_TX_QUEUE_SIZE];
    HostReplyCache_t replies[HOST_REPLY_CACHE_SIZE];
    uint8_t joint_payload[HOST_JOINT_STATE_SIZE];
    uint8_t frame[HOST_FRAME_MAX_SIZE];
} Host_t;

void Host_Init(Host_t *host, const HostConfig_t *config, Arm_t *arm);
BaseType_t Host_BindUart(Host_t *host);
void Host_Transmit(Host_t *host, UartInstance_t *uart);

uint8_t Host_Crc8(const uint8_t *data, uint16_t length);
uint16_t Host_Crc16(const uint8_t *data, uint16_t length);

#endif

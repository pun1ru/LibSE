#ifndef _CAN_DRIVER_H_
#define _CAN_DRIVER_H_

#include "fdcan.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef struct {
	uint32_t tx_ok;          // 发送成功计数
	uint32_t tx_fail;        // 发送失败计数
	uint16_t last_fail_id;   // 最后一次失败的目标 ID
	uint32_t err_callback;   // ErrorCallback 触发次数
	uint32_t err_reinit;     // 重初始化次数
} CANTxMonitor;

typedef struct {
    FDCAN_HandleTypeDef *can;
    CANTxMonitor monitor;
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
    uint32_t rx_fifo;
    void (*on_receive)(FDCAN_HandleTypeDef *can, uint16_t id,
                       const uint8_t *data, uint8_t length);
} CanInstance_t;

void CAN_RegisterInstances(CanInstance_t *instances, uint8_t count);

//can发送帧结构体
typedef struct {
    FDCAN_HandleTypeDef *can;
    uint16_t id;
    uint8_t data[8];
} CanTxFrame_t;

typedef struct {
    uint16_t id;
    uint8_t length;
    uint8_t data[8];
} CanRxFrame_t;

void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
/* 在启动发送任务前创建三个队列；按 hcan 分流，队列满时返回 pdFAIL。 */
BaseType_t CANTransmit_QueueInit(void);
BaseType_t CANReceive_QueueInit(void);
BaseType_t CANTransmit_Enqueue(FDCAN_HandleTypeDef *hcan, uint16_t std_id,
                               const uint8_t data[8]);
void CANTransmit_QueueTask(void *argument);
void CANReceive_QueueTask(void *argument);
void CANTransmit_U8(FDCAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t aData[]);
void CANTransmit_I16(FDCAN_HandleTypeDef *hcan, uint32_t std_id, int16_t output1, int16_t output2, int16_t output3, int16_t output4);
void CANInitialize(FDCAN_HandleTypeDef *hcan);
void CANReceiveEnable(FDCAN_HandleTypeDef *hcan);

#endif

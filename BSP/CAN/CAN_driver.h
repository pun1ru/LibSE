#ifndef _CAN_DRIVER_H_
#define _CAN_DRIVER_H_

#include <stdint.h>

#include "fdcan.h"
#include "FreeRTOS.h"
#include "queue.h"

#define CAN_TX_QUEUE_LENGTH 16U
#define CAN_RX_QUEUE_LENGTH 32U
#define CAN_STANDARD_ID_MAX 0x7FFU
#define CAN_FRAME_DATA_MAX 8U

typedef struct {
    uint32_t tx_ok;
    uint32_t tx_fail;
    uint16_t last_fail_id;
    uint32_t err_callback;
    uint32_t err_reinit;
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

typedef struct {
    FDCAN_HandleTypeDef *can;
    uint16_t id;
    uint8_t data[CAN_FRAME_DATA_MAX];
    uint8_t length;
} CanTxFrame_t;

typedef struct {
    uint16_t id;
    uint8_t length;
    uint8_t data[CAN_FRAME_DATA_MAX];
} CanRxFrame_t;

void CAN_RegisterInstances(CanInstance_t *instances, uint8_t count);
CanInstance_t *CAN_GetInstances(uint8_t *count);
uint8_t CAN_GetInstanceCount(void);

void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id,
                         const uint8_t *data, uint32_t len);

BaseType_t CANTransmit_QueueInit(void);
BaseType_t CANReceive_QueueInit(void);
BaseType_t CANTransmit_Enqueue(FDCAN_HandleTypeDef *hcan, uint16_t std_id,
                               const uint8_t data[CAN_FRAME_DATA_MAX]);
BaseType_t CANTransmit_EnqueueLength(FDCAN_HandleTypeDef *hcan,
                                     uint16_t std_id, const uint8_t *data,
                                     uint8_t len);
void CANTransmit_QueueTask(void *argument);
void CANTransmit_U8(FDCAN_HandleTypeDef *hcan, uint32_t std_id,
                    uint8_t aData[]);
void CANTransmit_I16(FDCAN_HandleTypeDef *hcan, uint32_t std_id,
                     int16_t output1, int16_t output2, int16_t output3,
                     int16_t output4);

/* Compatibility entry points used by older task code. */
void CANInitialize(FDCAN_HandleTypeDef *hcan);
void CANReceiveEnable(FDCAN_HandleTypeDef *hcan);

#endif

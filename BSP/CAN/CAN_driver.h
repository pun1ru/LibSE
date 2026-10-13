#ifndef _CAN_DRIVER_H_
#define _CAN_DRIVER_H_

#include <stdint.h>

#include "fdcan.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define CAN_TX_QUEUE_LENGTH 16U
#define CAN_RX_QUEUE_LENGTH 32U
#define CAN_STANDARD_ID_MAX 0x7FFU
#define CAN_FRAME_DATA_MAX 8U
#define CAN_RECOVER_REASON_BUS_OFF     1U
#define CAN_RECOVER_REASON_TX_STUCK    2U
#define CAN_RECOVER_REASON_NOT_RUNNING 3U

typedef struct {
    uint32_t tx_ok;
    uint32_t tx_fail;
    uint16_t last_fail_id;
    uint32_t err_callback;
    uint32_t err_status_callback;
    uint32_t err_reinit;
    uint32_t tx_fifo_capacity;
    uint32_t tx_fifo_free;
    uint32_t tx_fifo_used;
    uint32_t tx_fifo_min_free;
    uint32_t tx_fifo_full_count;
    uint32_t rx_fifo0_capacity;
    uint32_t rx_fifo0_fill;
    uint32_t rx_fifo0_free;
    uint32_t rx_fifo0_peak_fill;
    uint32_t rx_fifo1_capacity;
    uint32_t rx_fifo1_fill;
    uint32_t rx_fifo1_free;
    uint32_t rx_fifo1_peak_fill;
    uint32_t last_error_code;
    uint32_t data_last_error_code;
    uint32_t activity;
    uint32_t error_passive;
    uint32_t warning;
    uint32_t bus_off;
    uint32_t tx_error_cnt;
    uint32_t rx_error_cnt;
    uint32_t rx_error_passive;
    uint32_t hal_error_code;
    uint32_t recover_success;
    uint32_t recover_fail;
    uint32_t last_recover_reason;
    uint32_t last_recover_error_code;
    uint32_t last_recover_hal_error_code;
    uint32_t last_recover_tx_error_cnt;
    TickType_t last_recover_tick;
} CANTxMonitor;

typedef struct {
    FDCAN_HandleTypeDef *can;
    CANTxMonitor monitor;
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
    SemaphoreHandle_t tx_mutex;
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
void CAN_MonitorPoll(void);
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

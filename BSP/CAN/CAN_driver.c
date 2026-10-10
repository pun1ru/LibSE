#include "CAN_driver.h"

#include "task.h"

#include <stddef.h>
#include <string.h>

static CanInstance_t *can_instances;
static uint8_t can_instance_count;

void CAN_RegisterInstances(CanInstance_t *instances, uint8_t count)
{
    can_instances = instances;
    can_instance_count = (instances != NULL) ? count : 0U;
}

CanInstance_t *CAN_GetInstances(uint8_t *count)
{
    if (count != NULL) {
        *count = can_instance_count;
    }
    return can_instances;
}

uint8_t CAN_GetInstanceCount(void)
{
    return can_instance_count;
}

static CanInstance_t *CAN_InstanceFor(FDCAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || can_instances == NULL) {
        return NULL;
    }

    for (uint8_t i = 0U; i < can_instance_count; ++i) {
        if (can_instances[i].can == hcan) {
            return &can_instances[i];
        }
    }
    return NULL;
}

static QueueHandle_t CANTransmit_QueueFor(FDCAN_HandleTypeDef *hcan)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    return (instance != NULL) ? instance->tx_queue : NULL;
}

static void CAN_ConfigFilter(const CanInstance_t *instance)
{
    if (instance == NULL || instance->can == NULL) {
        return;
    }

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    filter.FilterConfig = (instance->rx_fifo == FDCAN_RX_FIFO0) ?
                          FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;

    (void)HAL_FDCAN_ConfigFilter(instance->can, &filter);
    (void)HAL_FDCAN_ConfigGlobalFilter(instance->can,
                                       FDCAN_REJECT, FDCAN_REJECT,
                                       FDCAN_FILTER_REMOTE,
                                       FDCAN_FILTER_REMOTE);
}

void can_filter_init(void)
{
    if (can_instances == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < can_instance_count; ++i) {
        CAN_ConfigFilter(&can_instances[i]);
    }
}

void CANReceiveEnable(FDCAN_HandleTypeDef *hcan)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    if (instance == NULL || instance->can == NULL) {
        return;
    }

    uint32_t notification = (instance->rx_fifo == FDCAN_RX_FIFO0) ?
                            FDCAN_IT_RX_FIFO0_NEW_MESSAGE :
                            FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
    (void)HAL_FDCAN_ActivateNotification(instance->can, notification, 0U);
}

void CANInitialize(FDCAN_HandleTypeDef *hcan)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    if (instance == NULL || instance->can == NULL) {
        return;
    }

    CAN_ConfigFilter(instance);
    if (HAL_FDCAN_Start(instance->can) != HAL_OK) {
        instance->monitor.err_reinit++;
        return;
    }
    CANReceiveEnable(instance->can);
}

void can_bsp_init(void)
{
    if (can_instances == NULL) {
        return;
    }

    can_filter_init();
    for (uint8_t i = 0U; i < can_instance_count; ++i) {
        CanInstance_t *instance = &can_instances[i];
        if (instance->can == NULL) {
            continue;
        }
        if (HAL_FDCAN_Start(instance->can) != HAL_OK) {
            instance->monitor.err_reinit++;
            continue;
        }
        CANReceiveEnable(instance->can);
    }
}

uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id,
                         const uint8_t *data, uint32_t len)
{
    CanInstance_t *instance = CAN_InstanceFor(hfdcan);
    if (instance != NULL && (id > CAN_STANDARD_ID_MAX || len > CAN_FRAME_DATA_MAX ||
                             (len != 0U && data == NULL))) {
        instance->monitor.tx_fail++;
        instance->monitor.last_fail_id = id;
        return 1U;
    }
    if (hfdcan == NULL || id > CAN_STANDARD_ID_MAX || len > CAN_FRAME_DATA_MAX ||
        (len != 0U && data == NULL)) {
        return 1U;
    }

    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    /* The HAL expects the DLC code (0..8), not a value shifted into a register. */
    header.DataLength = len;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header,
                                      (uint8_t *)(uintptr_t)data) != HAL_OK) {
        if (instance != NULL) {
            instance->monitor.tx_fail++;
            instance->monitor.last_fail_id = id;
        }
        return 1U;
    }
    if (instance != NULL) {
        instance->monitor.tx_ok++;
    }
    return 0U;
}

BaseType_t CANTransmit_QueueInit(void)
{
    if (can_instances == NULL || can_instance_count == 0U) {
        return pdFAIL;
    }

    for (uint8_t i = 0U; i < can_instance_count; ++i) {
        if (can_instances[i].tx_queue == NULL) {
            can_instances[i].tx_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH,
                                                     sizeof(CanTxFrame_t));
        }
        if (can_instances[i].tx_queue == NULL) {
            return pdFAIL;
        }
    }
    return pdPASS;
}

BaseType_t CANReceive_QueueInit(void)
{
    if (can_instances == NULL || can_instance_count == 0U) {
        return pdFAIL;
    }

    for (uint8_t i = 0U; i < can_instance_count; ++i) {
        if (can_instances[i].rx_queue == NULL) {
            can_instances[i].rx_queue = xQueueCreate(CAN_RX_QUEUE_LENGTH,
                                                     sizeof(CanRxFrame_t));
        }
        if (can_instances[i].rx_queue == NULL) {
            return pdFAIL;
        }
    }
    return pdPASS;
}

static void CANReceive_FromIsr(FDCAN_HandleTypeDef *hcan, uint32_t fifo)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    BaseType_t task_woken = pdFALSE;
    FDCAN_RxHeaderTypeDef header;
    CanRxFrame_t frame;
    uint8_t rx_data[64];

    if (hcan == NULL) {
        return;
    }

    /* Drain the hardware FIFO even when registration or queue creation failed. */
    while (HAL_FDCAN_GetRxFifoFillLevel(hcan, fifo) != 0U) {
        if (HAL_FDCAN_GetRxMessage(hcan, fifo, &header, rx_data) != HAL_OK) {
            break;
        }

        if (instance == NULL || instance->rx_queue == NULL ||
            header.IdType != FDCAN_STANDARD_ID ||
            header.RxFrameType != FDCAN_DATA_FRAME ||
            header.DataLength > FDCAN_DLC_BYTES_8) {
            continue;
        }

        frame.id = (uint16_t)header.Identifier;
        frame.length = (uint8_t)header.DataLength;
        memset(frame.data, 0, sizeof(frame.data));
        if (frame.length != 0U) {
            memcpy(frame.data, rx_data, frame.length);
        }
        (void)xQueueSendFromISR(instance->rx_queue, &frame, &task_woken);
    }
    portYIELD_FROM_ISR(task_woken);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t flags)
{
    if ((flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        CANReceive_FromIsr(hcan, FDCAN_RX_FIFO0);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hcan, uint32_t flags)
{
    if ((flags & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U) {
        CANReceive_FromIsr(hcan, FDCAN_RX_FIFO1);
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hcan)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    if (instance != NULL) {
        instance->monitor.err_callback++;
    }
}

BaseType_t CANTransmit_EnqueueLength(FDCAN_HandleTypeDef *hcan,
                                     uint16_t std_id, const uint8_t *data,
                                     uint8_t len)
{
    QueueHandle_t queue = CANTransmit_QueueFor(hcan);
    if (queue == NULL || std_id > CAN_STANDARD_ID_MAX ||
        len > CAN_FRAME_DATA_MAX || (len != 0U && data == NULL)) {
        return pdFAIL;
    }

    CanTxFrame_t frame = {0};
    frame.can = hcan;
    frame.id = std_id;
    frame.length = len;
    if (len != 0U) {
        memcpy(frame.data, data, len);
    }
    return xQueueSend(queue, &frame, 0U);
}

BaseType_t CANTransmit_Enqueue(FDCAN_HandleTypeDef *hcan, uint16_t std_id,
                               const uint8_t data[CAN_FRAME_DATA_MAX])
{
    return CANTransmit_EnqueueLength(hcan, std_id, data,
                                     CAN_FRAME_DATA_MAX);
}

void CANTransmit_QueueTask(void *argument)
{
    FDCAN_HandleTypeDef *hcan = (FDCAN_HandleTypeDef *)argument;
    QueueHandle_t queue = CANTransmit_QueueFor(hcan);
    CanTxFrame_t frame;

    if (queue == NULL) {
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        if (xQueueReceive(queue, &frame, portMAX_DELAY) == pdPASS) {
            (void)fdcanx_send_data(frame.can, frame.id, frame.data,
                                   frame.length);
        }
    }
}

void CANTransmit_U8(FDCAN_HandleTypeDef *hcan, uint32_t std_id,
                    uint8_t data[])
{
    (void)fdcanx_send_data(hcan, (uint16_t)std_id, data,
                           CAN_FRAME_DATA_MAX);
}

void CANTransmit_I16(FDCAN_HandleTypeDef *hcan, uint32_t std_id,
                     int16_t output1, int16_t output2, int16_t output3,
                     int16_t output4)
{
    int16_t values[4] = {output1, output2, output3, output4};
    uint8_t data[CAN_FRAME_DATA_MAX];
    for (uint8_t i = 0U; i < 4U; ++i) {
        data[2U * i] = (uint8_t)((uint16_t)values[i] >> 8U);
        data[2U * i + 1U] = (uint8_t)values[i];
    }
    (void)fdcanx_send_data(hcan, (uint16_t)std_id, data,
                           CAN_FRAME_DATA_MAX);
}

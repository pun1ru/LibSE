#include "CAN_driver.h"
#include "task.h"

#include <stddef.h>
#include <string.h>


static CanInstance_t *can_instances;
static uint8_t can_instance_count;

//注册can实例数量和指针
void CAN_RegisterInstances(CanInstance_t *instances, uint8_t count)
{
    can_instances = instances;
    can_instance_count = count;
}

//输入can句柄，返回对应的can实例指针
static CanInstance_t *CAN_InstanceFor(FDCAN_HandleTypeDef *hcan)
{
    for (uint8_t i = 0U; i < can_instance_count; ++i) {
        if (can_instances[i].can == hcan) {
            return &can_instances[i];
        }
    }
    return NULL;
}
//输入can句柄，返回发送队列的句柄
static QueueHandle_t CANTransmit_QueueFor(FDCAN_HandleTypeDef *hcan)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    return instance != NULL ? instance->tx_queue : NULL;
}

//can初始化，包含滤波器初始化，使能can，使能can中断,根据分配的fifo使能对应的rx中断
void can_bsp_init(void)
{
    can_filter_init();
    for (uint8_t i = 0U; i < can_instance_count; ++i) {
        CanInstance_t *instance = &can_instances[i];
        HAL_FDCAN_Start(instance->can);
        uint32_t notification = instance->rx_fifo == FDCAN_RX_FIFO0 ?
                                FDCAN_IT_RX_FIFO0_NEW_MESSAGE :
                                FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
        HAL_FDCAN_ActivateNotification(instance->can, notification, 0U);
    }
}
//can初始化的子函数，滤波器初始化，掩码模式过滤ID2为零不过滤
void can_filter_init(void)
{
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;

    for (uint8_t i = 0U; i < can_instance_count; ++i) {
        CanInstance_t *instance = &can_instances[i];
        filter.FilterConfig = instance->rx_fifo == FDCAN_RX_FIFO0 ?
                              FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
        HAL_FDCAN_ConfigFilter(instance->can, &filter);
        HAL_FDCAN_ConfigGlobalFilter(instance->can, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    }
}
//can发送数据函数，返回0表示发送成功，返回1表示发送失败
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id,
                         uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef header = {0};
    CanInstance_t *instance = CAN_InstanceFor(hfdcan);
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = len << 16;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, data) != HAL_OK) {
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
//can发送队列初始化函数，创建发送队列
BaseType_t CANTransmit_QueueInit(void)
{
    for (uint8_t i = 0U; i < can_instance_count; ++i) {
        if (can_instances[i].tx_queue == NULL) {
            can_instances[i].tx_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH,
                                                     sizeof(CanTxFrame_t));
        }
    }
    return can_instance_count != 0U ? pdPASS : pdFAIL;
}
//can接收队列初始化函数，创建接收队列
BaseType_t CANReceive_QueueInit(void)
{
    for (uint8_t i = 0U; i < can_instance_count; ++i) {
        if (can_instances[i].rx_queue == NULL) {
            can_instances[i].rx_queue = xQueueCreate(CAN_RX_QUEUE_LENGTH,
                                                     sizeof(CanRxFrame_t));
        }
        if (can_instances[i].rx_queue == NULL) {
            return pdFAIL;
        }
    }
    return can_instance_count != 0U ? pdPASS : pdFAIL;
}

//can中断处理函数，主要把数据搬到接收队列
static void CANReceive_FromIsr(FDCAN_HandleTypeDef *hcan, uint32_t fifo)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    BaseType_t task_woken = pdFALSE;
    FDCAN_RxHeaderTypeDef header;
    CanRxFrame_t frame;
    uint8_t rx_data[64];
    //一次全部读完fifo不留帧
    while (HAL_FDCAN_GetRxFifoFillLevel(hcan, fifo) != 0U) {
        if (HAL_FDCAN_GetRxMessage(hcan, fifo, &header, rx_data) != HAL_OK) {
            break;
        }

        if (header.IdType == FDCAN_STANDARD_ID &&
            header.RxFrameType == FDCAN_DATA_FRAME &&
            header.DataLength <= FDCAN_DLC_BYTES_8) {
            frame.id = (uint16_t)header.Identifier;
            frame.length = (uint8_t)header.DataLength;
            memcpy(frame.data, rx_data, frame.length);
            (void)xQueueSendFromISR(instance->rx_queue, &frame, &task_woken);
        }
    }
    //ISR返回到更高优先级任务如果有
    portYIELD_FROM_ISR(task_woken);
}
//fifo0,1中断回调函数
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

//发送消息进队
BaseType_t CANTransmit_Enqueue(FDCAN_HandleTypeDef *hcan, uint16_t std_id,
                               const uint8_t data[8])
{
    QueueHandle_t queue = CANTransmit_QueueFor(hcan);
    CanTxFrame_t frame = {.can = hcan, .id = std_id};
    memcpy(frame.data, data, sizeof(frame.data));
    return xQueueSend(queue, &frame, 0U);
}

void CANTransmit_QueueTask(void *argument)
{
    QueueHandle_t queue = CANTransmit_QueueFor((FDCAN_HandleTypeDef *)argument);
    CanTxFrame_t frame;
    for (;;) {
        if (xQueueReceive(queue, &frame, portMAX_DELAY) == pdPASS) {
            (void)fdcanx_send_data(frame.can, frame.id, frame.data, sizeof(frame.data));
        }
    }
}

void CANTransmit_U8(FDCAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t data[])
{
    (void)fdcanx_send_data(hcan, (uint16_t)std_id, data, 8U);
}

void CANTransmit_I16(FDCAN_HandleTypeDef *hcan, uint32_t std_id,
                     int16_t output1, int16_t output2, int16_t output3,
                     int16_t output4)
{
    int16_t values[4] = {output1, output2, output3, output4};
    uint8_t data[8];
    for (uint8_t i = 0U; i < 4U; ++i) {
        data[2U * i] = (uint8_t)((uint16_t)values[i] >> 8);
        data[2U * i + 1U] = (uint8_t)values[i];
    }
    (void)fdcanx_send_data(hcan, (uint16_t)std_id, data, sizeof(data));
}

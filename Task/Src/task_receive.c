#include "task_receive.h"

#include "CAN_driver.h"
#include "FreeRTOS.h"
#include "monitor.h"
#include "referee.h"
#include "remote_control.h"
#include "task.h"
#include "task_define.h"
#include "task_system.h"

#include <stddef.h>
#include <string.h>

#define TASK_UART_PACKET_SIZE UART_DMA_BUFFER_SIZE

/*--------------------------------------回调函数定义--------------------------------------*/
static void TaskReceive_RCInput(const uint8_t *data, uint16_t size)
{
    RC_Remote_Input(&rc_receiver, data, size);
}

static void TaskReceive_RCError(void)
{
    RC_Remote_OnError(&rc_receiver);
}

static void TaskReceive_RefereeInput(const uint8_t *data, uint16_t size)
{
    Referee_Input(&referee_receiver, data, size);
}

static void TaskReceive_RefereeError(void)
{
    Referee_Reset(&referee_receiver);
}

static void TaskReceive_DispatchMotorFeedback(const TaskMotorBinding_t *bindings,
                                              uint8_t binding_count, uint16_t id,
                                              const uint8_t *data, uint8_t length)
{
    if (length != CAN_FRAME_DATA_MAX)
    {
        return;
    }

    for (uint8_t index = 0U; index < binding_count; ++index)
    {
        const TaskMotorBinding_t *binding = &bindings[index];
        if (id == binding->feedback_id)
        {
            binding->motor->motorops.getfeedback(binding->motor, data, id);
            break;
        }
    }
}

static void TaskReceive_CAN1Callback(FDCAN_HandleTypeDef *can, uint16_t id,
                                      const uint8_t *data, uint8_t length)
{
    (void)can;
    TaskReceive_DispatchMotorFeedback(can1MotorBindings,
                                      TASK_CAN1_MOTOR_BINDING_COUNT,
                                      id, data, length);
}

static void TaskReceive_CAN2Callback(FDCAN_HandleTypeDef *can, uint16_t id,
                                      const uint8_t *data, uint8_t length)
{
    (void)can;
    TaskReceive_DispatchMotorFeedback(can2MotorBindings,
                                      TASK_CAN2_MOTOR_BINDING_COUNT,
                                      id, data, length);
}

static void TaskReceive_CAN3Callback(FDCAN_HandleTypeDef *can, uint16_t id,
                                      const uint8_t *data, uint8_t length)
{
    (void)can;
    TaskReceive_DispatchMotorFeedback(can3MotorBindings,
                                      TASK_CAN3_MOTOR_BINDING_COUNT,
                                      id, data, length);
}
/*--------------------------------------初始化协议--------------------------------------*/
static void TaskReceive_InitProtocols(void)
{
    //初始化
    Referee_Init(&referee_receiver);
    RC_Init(&rc_receiver);
    // CAN callbacks
    can_instances[0].on_receive = TaskReceive_CAN1Callback;
    can_instances[1].on_receive = TaskReceive_CAN2Callback;
    can_instances[2].on_receive = TaskReceive_CAN3Callback;
    //定义函数指针映射到对应的回调函数
    //UART回调
    uart_instances[TASK_UART_HUART5_INDEX].on_packet = TaskReceive_RCInput;
    uart_instances[TASK_UART_HUART5_INDEX].on_error = TaskReceive_RCError;
    uart_instances[TASK_UART_HUART1_INDEX].on_packet = TaskReceive_RefereeInput;
    uart_instances[TASK_UART_HUART1_INDEX].on_error = TaskReceive_RefereeError;
}
/*--------------------------------------默认回调函数--------------------------------------*/
void CANReceive_Callback(FDCAN_HandleTypeDef *can, uint16_t id,
                         const uint8_t *data, uint8_t length)
{
}
void UARTReceive_Callback(UartInstance_t *uart, const uint8_t *data,
                          uint16_t length)
{
}
/*---------------------------------------初始化can/uart-------------------------------------*/
static void TaskReceive_BindUartTask(void)
{
    for (uint8_t i = 0U; i < TASK_UART_INSTANCE_COUNT; ++i) {
        UartInstance_t *uart = &uart_instances[i];
        (void)UART_SetNotifyTask(uart, xTaskGetCurrentTaskHandle());
    }
}

/*---------------------------------------uart数据处理-------------------------------------*/
static void TaskReceive_ProcessUartInstance(UartInstance_t *uart)
{
    uint8_t packet[TASK_UART_PACKET_SIZE];
    uint16_t packet_size;

    while (UART_RxPop(uart, packet, sizeof(packet), &packet_size) == pdTRUE) {
        if (uart->on_packet != NULL) {
            uart->on_packet(packet, packet_size);
        } else {
            UARTReceive_Callback(uart, packet, packet_size);
        }
    }
}

static void TaskReceive_ProcessUart(UartInstance_t *instances,
                                    uint8_t instance_count)
{
    for (uint8_t i = 0U; i < instance_count; ++i) {
        UartInstance_t *uart = &instances[i];
        EventBits_t events = UART_GetEvents(uart, pdTRUE);

        if ((events & UART_EVENT_RX_READY) != 0U ||
            UART_RxAvailable(uart) != 0U) {
            TaskReceive_ProcessUartInstance(uart);
        }
        if ((events & (UART_EVENT_RX_ERROR |
                      UART_EVENT_RX_OVERFLOW)) != 0U) {
            if (uart->on_error != NULL) {
                uart->on_error();
            }
            Monitor_TaskError(MONITOR_TASK_RECEIVE);
        }
    }
}
/*--------------------------------------CAN数据处理--------------------------------------*/
static void TaskReceive_ProcessCan(QueueSetHandle_t queues)
{
    CanRxFrame_t frame;
    QueueSetMemberHandle_t ready;
    //xQueueSelectFromSet返回队列集中有数据的队列句柄/null
    while ((ready = xQueueSelectFromSet(queues, 0U)) != NULL) {
        if (xQueueReceive(ready, &frame, 0U) != pdPASS) {
            continue;
        }
        //查找对应的队列调用对应的回调函数
        for (uint8_t i = 0U; i < TASK_CAN_INSTANCE_COUNT; ++i) {
            if (ready == can_instances[i].rx_queue) {
                //定义每个接收队列自己的回调函数,不然就是默认
                if (can_instances[i].on_receive != NULL) {
                    can_instances[i].on_receive(can_instances[i].can, frame.id,
                                                frame.data, frame.length);
                } else {
                    CANReceive_Callback(can_instances[i].can, frame.id,
                                        frame.data, frame.length);
                }
                break;
            }
        }
    }
}
/*--------------------------------------接收任务主体--------------------------------------*/
void ReceiveTask(void *argument)
{
    uint32_t notification_value;

    (void)argument;

    TaskReceive_InitProtocols();
    TaskReceive_BindUartTask();

    while(1) {
        //CAN/UART处理函数,调用回调函数
        
        TaskReceive_ProcessCan(receive_can_queues);

        TaskReceive_ProcessUart(uart_instances,
                                TASK_UART_INSTANCE_COUNT);

        Monitor_TaskBeat(MONITOR_TASK_RECEIVE);
        notification_value = 0U;
        (void)xTaskNotifyWait(0U, (uint32_t)UART_EVENT_ALL,
                              &notification_value,
                              pdMS_TO_TICKS(TASK_RECEIVE_PERIOD_MS));
    }
}

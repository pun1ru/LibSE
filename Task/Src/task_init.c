#include "task_init.h"

#include "CAN_driver.h"
#include "bsp_dwt.h"
#include "fdcan.h"
#include "monitor.h"
#include "task_define.h"
#include "task_ins.h"
#include "task_monitor.h"
#include "task_receive.h"
#include "task_system.h"
#include "task_transmit.h"
#include "usart.h"

#include "main.h"
#include "event_groups.h"
#include "task.h"

#include <string.h>

static void UARTRegister(UartInstance_t *uart, UART_HandleTypeDef *huart,
                         EventBits_t rx_bit, EventBits_t error_bit)
{
    memset(uart, 0, sizeof(*uart));
    uart->huart = huart;
    UART_Register(uart);
    UART_SetEventGroup(uart, UartEventGroup, rx_bit, error_bit);
}
void CANInit(void)
{
    memset(can_instances, 0, sizeof(can_instances));

    can_instances[0].can = &hfdcan1;
    can_instances[0].rx_fifo = FDCAN_RX_FIFO0;
    can_instances[0].on_receive = TaskReceive_CANCallback;

    can_instances[1].can = &hfdcan2;
    can_instances[1].rx_fifo = FDCAN_RX_FIFO1;
    can_instances[1].on_receive = TaskReceive_CANCallback;

    can_instances[2].can = &hfdcan3;
    can_instances[2].rx_fifo = FDCAN_RX_FIFO1;
    can_instances[2].on_receive = TaskReceive_CANCallback;

    CAN_RegisterInstances(can_instances, TASK_CAN_INSTANCE_COUNT);
    if (CANTransmit_QueueInit() != pdPASS ||
        CANReceive_QueueInit() != pdPASS) {
        Error_Handler();
    }

    can_bsp_init();
}

static void UARTInit(void)
{
    UartEventGroup = xEventGroupCreate();

    UART_EventGroup = UartEventGroup;

    UARTRegister(&uart_instances[TASK_UART_HUART5_INDEX], &huart5,
                 TASK_UART_EVENT_HUART5_RX, TASK_UART_EVENT_HUART5_ERROR);
    UARTRegister(&uart_instances[TASK_UART_HUART7_INDEX], &huart7,
                 TASK_UART_EVENT_HUART7_RX, TASK_UART_EVENT_HUART7_ERROR);
    UARTRegister(&uart_instances[TASK_UART_HUART1_INDEX], &huart1,
                 TASK_UART_EVENT_HUART1_RX, TASK_UART_EVENT_HUART1_ERROR);
    UARTRegister(&uart_instances[TASK_UART_HUART2_INDEX], &huart2,
                 TASK_UART_EVENT_HUART2_RX, TASK_UART_EVENT_HUART2_ERROR);
    UARTRegister(&uart_instances[TASK_UART_HUART3_INDEX], &huart3,
                 TASK_UART_EVENT_HUART3_RX, TASK_UART_EVENT_HUART3_ERROR);
    UARTRegister(&uart_instances[TASK_UART_HUART10_INDEX], &huart10,
                 TASK_UART_EVENT_HUART10_RX, TASK_UART_EVENT_HUART10_ERROR);
}

void CommunicateInit(void)
{
    CANInit();
    UARTInit();
}

static BaseType_t CreateTask(TaskFunction_t function, const char *name,
                             uint16_t stack_size, UBaseType_t priority,
                             void *argument, TaskHandle_t *handle)
{
    return xTaskCreate(function, name, stack_size, argument, priority, handle);
}

void InitTask(void const *argument)
{
    (void)argument;

    DWT_Init(TASK_CPU_FREQUENCY_MHZ);
    Monitor_Init();
    CommunicateInit();

    if (CreateTask(MonitorTask, "MonitorTask", TASK_MONITOR_STACK_SIZE,
                   TASK_MONITOR_PRIORITY, NULL, &MonitorTaskHandle) != pdPASS ||
        CreateTask(ReceiveTask, "ReceiveTask", TASK_RECEIVE_STACK_SIZE,
                   TASK_RECEIVE_PRIORITY, can_instances, &ReceiveTaskHandle) != pdPASS ||
        CreateTask(TransmitTask, "TransmitTask", TASK_TRANSMIT_STACK_SIZE,
                   TASK_TRANSMIT_PRIORITY, can_instances, &TransmitTaskHandle) != pdPASS ||
        CreateTask(INS_Task, "INSTask", TASK_INS_STACK_SIZE,
                   TASK_INS_PRIORITY, NULL, &INSTaskHandle) != pdPASS) {
        Error_Handler();
    }
}

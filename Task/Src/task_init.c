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
#include "../../Drivers/WS2812/WS2812.h"
#include "spi.h"
#include "usart.h"
#include "bsp_buzzer.h"
#include "main.h"
#include "event_groups.h"
#include "task.h"

#include <string.h>
/*--------------------------------------通信初始化--------------------------------------*/
static void UARTRegister(UartInstance_t *uart, UART_HandleTypeDef *huart,
                         EventBits_t rx_bit, EventBits_t error_bit)
{
    memset(uart, 0, sizeof(*uart));
    uart->huart = huart;
    UART_Register(uart);
    UART_SetEventGroup(uart, UartEventGroup, rx_bit, error_bit);
}

static void CANQueueSetsInit(void)
{
    receive_can_queues = xQueueCreateSet((UBaseType_t)TASK_CAN_INSTANCE_COUNT *(UBaseType_t)CAN_RX_QUEUE_LENGTH);
    transmit_can_queues = xQueueCreateSet((UBaseType_t)TASK_CAN_INSTANCE_COUNT *(UBaseType_t)CAN_TX_QUEUE_LENGTH);

    for (uint8_t i = 0U; i < TASK_CAN_INSTANCE_COUNT; ++i) {
        xQueueAddToSet(can_instances[i].rx_queue,receive_can_queues);
        xQueueAddToSet(can_instances[i].tx_queue,transmit_can_queues);
          
    }
}
void CANInit(void)
{
    memset(can_instances, 0, sizeof(can_instances));

    can_instances[0].can = &hfdcan1;
    can_instances[0].rx_fifo = FDCAN_RX_FIFO0;

    can_instances[1].can = &hfdcan2;
    can_instances[1].rx_fifo = FDCAN_RX_FIFO1;

    can_instances[2].can = &hfdcan3;
    can_instances[2].rx_fifo = FDCAN_RX_FIFO1;

    CAN_RegisterInstances(can_instances, TASK_CAN_INSTANCE_COUNT);
    CANTransmit_QueueInit();
    CANReceive_QueueInit();
    CANQueueSetsInit();
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

    for (uint8_t i = 0U; i < TASK_UART_INSTANCE_COUNT; ++i) {
        UART_StartReceiveEx(&uart_instances[i]);
    }
}

void CommunicateInit(void)
{
    CANInit();
    UARTInit();
}
/*--------------------------------------任务初始化--------------------------------------*/
void InitTask(void *argument)
{
    taskENTER_CRITICAL();
    DWT_Init(TASK_CPU_FREQUENCY_MHZ);
    Monitor_Init();
    CommunicateInit();
    TaskSystem_InitMotors();
    WS2812_SPI_Init(&hspi6);
    Buzzer_BeepOnce();
    xTaskCreate(MonitorTask, "MonitorTask", TASK_MONITOR_STACK_SIZE, NULL,
                TASK_MONITOR_PRIORITY, &MonitorTaskHandle);
    xTaskCreate(ReceiveTask, "ReceiveTask", TASK_RECEIVE_STACK_SIZE, NULL,
                TASK_RECEIVE_PRIORITY, &ReceiveTaskHandle);
    xTaskCreate(TransmitTask, "TransmitTask", TASK_TRANSMIT_STACK_SIZE, NULL,
                TASK_TRANSMIT_PRIORITY, &TransmitTaskHandle);
    xTaskCreate(INS_Task, "INSTask", TASK_INS_STACK_SIZE, NULL,
                TASK_INS_PRIORITY, &INSTaskHandle);
    taskEXIT_CRITICAL();
    vTaskDelete(NULL);
}

#include "task_system.h"

CanInstance_t can_instances[TASK_CAN_INSTANCE_COUNT];

BSP_UART_t uart5_instance;
BSP_UART_t uart7_instance;
BSP_UART_t uart1_instance;
BSP_UART_t uart2_instance;
BSP_UART_t uart3_instance;
BSP_UART_t uart10_instance;
EventGroupHandle_t UartEventGroup;
Referee_t referee_receiver;

TaskHandle_t MonitorTaskHandle;
TaskHandle_t ReceiveTaskHandle;
TaskHandle_t TransmitTaskHandle;
TaskHandle_t INSTaskHandle;

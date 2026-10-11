#include "task_system.h"

CanInstance_t can_instances[TASK_CAN_INSTANCE_COUNT];
QueueSetHandle_t receive_can_queues;
QueueSetHandle_t transmit_can_queues;

UartInstance_t uart_instances[TASK_UART_INSTANCE_COUNT];
EventGroupHandle_t UartEventGroup;

Referee_t referee_receiver;
RC_Remote_t rc_receiver;
INS_t board_ins;

TaskHandle_t MonitorTaskHandle;
TaskHandle_t ReceiveTaskHandle;
TaskHandle_t TransmitTaskHandle;
TaskHandle_t INSTaskHandle;

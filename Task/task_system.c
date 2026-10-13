#include "task_system.h"

CanInstance_t can_instances[TASK_CAN_INSTANCE_COUNT];
QueueSetHandle_t receive_can_queues;
QueueSetHandle_t transmit_can_queues;

UartInstance_t uart_instances[TASK_UART_INSTANCE_COUNT];
EventGroupHandle_t UartEventGroup;

Referee_t referee_receiver;
RC_Remote_t rc_receiver;

TaskHandle_t MonitorTaskHandle;
TaskHandle_t ReceiveTaskHandle;
TaskHandle_t TransmitTaskHandle;
TaskHandle_t INSTaskHandle;

INS_t board_ins;

/* 电机实例由使用者自行添加，并在初始化时填写对应 CAN 的绑定表 */
TaskMotorBinding_t can1MotorBindings[TASK_CAN1_MOTOR_BINDING_COUNT];
TaskMotorBinding_t can2MotorBindings[TASK_CAN2_MOTOR_BINDING_COUNT];
TaskMotorBinding_t can3MotorBindings[TASK_CAN3_MOTOR_BINDING_COUNT];

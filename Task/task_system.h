#ifndef TASK_SYSTEM_H
#define TASK_SYSTEM_H

#include "CAN_driver.h"
#include "uart_driver.h"
#include "referee.h"
#include "remote_control.h"
#include "task.h"
#include "ins.h"
#include "Motor.h"
#include "DMMotor.h"
#include "LKMotor.h"
#include "gimbal.h"

#define TASK_CAN_INSTANCE_COUNT       3U
#define TASK_CAN1_MOTOR_BINDING_COUNT  1U
#define TASK_CAN2_MOTOR_BINDING_COUNT  1U
#define TASK_CAN3_MOTOR_BINDING_COUNT  1U
#define TASK_UART_INSTANCE_COUNT      6U

typedef struct {
    uint16_t feedback_id;
    Motor_t *motor;
} TaskMotorBinding_t;

enum {
    TASK_UART_HUART5_INDEX = 0U,
    TASK_UART_HUART7_INDEX,
    TASK_UART_HUART1_INDEX,
    TASK_UART_HUART2_INDEX,
    TASK_UART_HUART3_INDEX,
    TASK_UART_HUART10_INDEX
};

#define TASK_UART_EVENT_HUART5_RX     ((EventBits_t)(1UL << 0U))
#define TASK_UART_EVENT_HUART5_ERROR  ((EventBits_t)(1UL << 1U))
#define TASK_UART_EVENT_HUART7_RX     ((EventBits_t)(1UL << 2U))
#define TASK_UART_EVENT_HUART7_ERROR  ((EventBits_t)(1UL << 3U))
#define TASK_UART_EVENT_HUART1_RX     ((EventBits_t)(1UL << 4U))
#define TASK_UART_EVENT_HUART1_ERROR  ((EventBits_t)(1UL << 5U))
#define TASK_UART_EVENT_HUART2_RX     ((EventBits_t)(1UL << 6U))
#define TASK_UART_EVENT_HUART2_ERROR  ((EventBits_t)(1UL << 7U))
#define TASK_UART_EVENT_HUART3_RX     ((EventBits_t)(1UL << 8U))
#define TASK_UART_EVENT_HUART3_ERROR  ((EventBits_t)(1UL << 9U))
#define TASK_UART_EVENT_HUART10_RX    ((EventBits_t)(1UL << 10U))
#define TASK_UART_EVENT_HUART10_ERROR ((EventBits_t)(1UL << 11U))

extern CanInstance_t can_instances[TASK_CAN_INSTANCE_COUNT];
extern QueueSetHandle_t receive_can_queues;
extern QueueSetHandle_t transmit_can_queues;
extern UartInstance_t uart_instances[TASK_UART_INSTANCE_COUNT];
extern EventGroupHandle_t UartEventGroup;
extern Referee_t referee_receiver;
extern RC_Remote_t rc_receiver;

extern TaskHandle_t MonitorTaskHandle;
extern TaskHandle_t ReceiveTaskHandle;
extern TaskHandle_t TransmitTaskHandle;
extern TaskHandle_t INSTaskHandle;
extern TaskHandle_t GimbalTaskHandle;

extern INS_t board_ins;
extern Gimbal_t gimbalInstance;
extern DMMotor_t gimbalYawMotor;
extern LKMotor_t gimbalPitchMotor;
extern TaskMotorBinding_t can1MotorBindings[TASK_CAN1_MOTOR_BINDING_COUNT];
extern TaskMotorBinding_t can2MotorBindings[TASK_CAN2_MOTOR_BINDING_COUNT];
extern TaskMotorBinding_t can3MotorBindings[TASK_CAN3_MOTOR_BINDING_COUNT];

#endif

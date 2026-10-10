#ifndef TASK_SYSTEM_H
#define TASK_SYSTEM_H

#include "CAN_driver.h"
#include "bsp_uart.h"
#include "referee.h"
#include "remote_control.h"
#include "task.h"

#define TASK_CAN_INSTANCE_COUNT       3U

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

extern BSP_UART_t uart5_instance;
extern BSP_UART_t uart7_instance;
extern BSP_UART_t uart1_instance;
extern BSP_UART_t uart2_instance;
extern BSP_UART_t uart3_instance;
extern BSP_UART_t uart10_instance;
extern EventGroupHandle_t UartEventGroup;
extern Referee_t referee_receiver;

extern TaskHandle_t MonitorTaskHandle;
extern TaskHandle_t ReceiveTaskHandle;
extern TaskHandle_t TransmitTaskHandle;
extern TaskHandle_t INSTaskHandle;

#endif

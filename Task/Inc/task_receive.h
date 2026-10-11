#ifndef TASK_RECEIVE_H
#define TASK_RECEIVE_H

#include "uart_driver.h"
#include "fdcan.h"

void ReceiveTask(void *argument);

void TaskReceive_CANCallback(FDCAN_HandleTypeDef *can, uint16_t id,
                             const uint8_t *data, uint8_t length);
void CANReceive_Callback(FDCAN_HandleTypeDef *can, uint16_t id,
                         const uint8_t *data, uint8_t length);
void UARTReceive_Callback(UartInstance_t *uart, const uint8_t *data,
                          uint16_t length);

#endif

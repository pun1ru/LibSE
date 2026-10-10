#ifndef REFEREE_H
#define REFEREE_H

#include "bsp_uart.h"
#include "judge_receive.h"

#define REFEREE_BUFFER_SIZE 320U

typedef struct {
    BSP_UART_t uart;
    uint8_t frame_buffer[REFEREE_BUFFER_SIZE];
    uint16_t frame_size;
} Referee_t;

/* Configure huart for the referee protocol baud rate before calling Init. */
void Referee_Init(Referee_t *referee, UART_HandleTypeDef *huart);
void Referee_Reset(Referee_t *referee);
void Referee_Input(Referee_t *referee, const uint8_t *data, uint16_t size);

#endif

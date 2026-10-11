#ifndef REFEREE_H
#define REFEREE_H

#include "uart_driver.h"
#include "judge_receive.h"

#define REFEREE_BUFFER_SIZE 320U

typedef struct {
    UartInstance_t uart;
    uint8_t frame_buffer[REFEREE_BUFFER_SIZE];
    uint16_t frame_size;
} Referee_t;

void Referee_Init(Referee_t *referee);
void Referee_Reset(Referee_t *referee);
void Referee_Input(Referee_t *referee, const uint8_t *data, uint16_t size);

#endif

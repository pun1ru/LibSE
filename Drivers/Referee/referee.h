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
/* 把解析器挂到串口口上，之后串口收到的包在中断里直接喂给 Referee_Input，出错时调 Referee_Reset
 * 在 Referee_Init 之后、串口启动接收之前调用，用法同 Motor_BindCan */
BaseType_t Referee_BindUart(Referee_t *referee, UartInstance_t *uart);
/* 在串口中断里执行 */
void Referee_Input(Referee_t *referee, const uint8_t *data, uint16_t size);

#endif

#ifndef BSP_UART_H
#define BSP_UART_H

#include "usart.h"

#define BSP_UART_DMA_BUFFER_SIZE 256U

typedef void (*BSP_UART_RxCallback_t)(void *context, const uint8_t *data,
                                      uint16_t size);

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t *rx_buffer[2];
    uint8_t *tx_buffer;
    uint16_t rx_capacity;
    uint8_t active_buffer;
    BSP_UART_RxCallback_t on_receive;
    void *context;
} BSP_UART_t;

void BSP_UART_Register(BSP_UART_t *uart);
void BSP_UART_StartReceive(BSP_UART_t *uart);
void BSP_UART_TransmitDMA(BSP_UART_t *uart, const uint8_t *data, uint16_t size);

#endif

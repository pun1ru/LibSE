#include "bsp_uart.h"

#include <stddef.h>
#include <string.h>

#define BSP_UART_MAX_INSTANCES 6U

static uint8_t uart_rx_dma[BSP_UART_MAX_INSTANCES][2][BSP_UART_DMA_BUFFER_SIZE]
    __attribute__((section(".bss.bsp_uart_dma"), aligned(32)));
static uint8_t uart_tx_dma[BSP_UART_MAX_INSTANCES][BSP_UART_DMA_BUFFER_SIZE]
    __attribute__((section(".bss.bsp_uart_dma"), aligned(32)));
static BSP_UART_t *uart_instances[BSP_UART_MAX_INSTANCES];
static uint8_t uart_instance_count;

static BSP_UART_t *BSP_UART_Find(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0U; i < uart_instance_count; ++i) {
        if (uart_instances[i]->huart == huart) {
            return uart_instances[i];
        }
    }
    return NULL;
}

void BSP_UART_Register(BSP_UART_t *uart)
{
    if (uart_instance_count < BSP_UART_MAX_INSTANCES) {
        uint8_t index = uart_instance_count++;
        uart_instances[index] = uart;
        uart->rx_buffer[0] = uart_rx_dma[index][0];
        uart->rx_buffer[1] = uart_rx_dma[index][1];
        uart->tx_buffer = uart_tx_dma[index];
        uart->rx_capacity = BSP_UART_DMA_BUFFER_SIZE;
    }
}

void BSP_UART_StartReceive(BSP_UART_t *uart)
{
    uint8_t *buffer = uart->rx_buffer[uart->active_buffer];
    SCB_InvalidateDCache_by_Addr(buffer, uart->rx_capacity);
    if (HAL_UARTEx_ReceiveToIdle_DMA(uart->huart, buffer,
                                     uart->rx_capacity) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(uart->huart->hdmarx, DMA_IT_HT);
    }
}

void BSP_UART_TransmitDMA(BSP_UART_t *uart, const uint8_t *data, uint16_t size)
{
    if (size > BSP_UART_DMA_BUFFER_SIZE ||
        uart->huart->gState != HAL_UART_STATE_READY) {
        return;
    }
    memcpy(uart->tx_buffer, data, size);
    SCB_CleanDCache_by_Addr((uint32_t *)uart->tx_buffer, size);
    (void)HAL_UART_Transmit_DMA(uart->huart, uart->tx_buffer, size);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    BSP_UART_t *uart = BSP_UART_Find(huart);
    if (uart == NULL) {
        return;
    }

    uint8_t *completed = uart->rx_buffer[uart->active_buffer];
    SCB_InvalidateDCache_by_Addr(completed, uart->rx_capacity);
    uart->active_buffer ^= 1U;
    BSP_UART_StartReceive(uart);
    if (uart->on_receive != NULL && size != 0U) {
        uart->on_receive(uart->context, completed, size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_t *uart = BSP_UART_Find(huart);
    if (uart != NULL) {
        BSP_UART_StartReceive(uart);
    }
}

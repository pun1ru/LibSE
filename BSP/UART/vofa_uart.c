#include "vofa_uart.h"

HAL_StatusTypeDef VofaJustFloat_Send(const VofaJustFloat_t *frame, UartInstance_t *uart)
{
    size_t size = VofaJustFloat_Size(frame);

    if (uart == NULL)
    {
        return HAL_ERROR;
    }
    if (size == 0U)
    {
        return HAL_OK;
    }
    if (size > UART_TX_BUFFER_SIZE)
    {
        return HAL_ERROR;
    }

    return UART_TransmitDMAEx(uart, VofaJustFloat_Data(frame), (uint16_t)size);
}

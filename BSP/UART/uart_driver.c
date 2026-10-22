#include "uart_driver.h"

#include <stddef.h>
#include <string.h>

void UartMonitor_Reset(UartMonitor_t *monitor)
{
    monitor->last_rx_size = 0U;
    monitor->last_rx_event = 0U;
    monitor->rx_event_count = 0U;
    monitor->rx_error_count = 0U;
    monitor->rx_overflow_count = 0U;
    monitor->tx_error_count = 0U;
}

void UartMonitor_RecordRx(UartMonitor_t *monitor, uint16_t size,
                          uint8_t event)
{
    monitor->last_rx_size = size;
    monitor->last_rx_event = event;
    monitor->rx_event_count++;
}

void UartMonitor_RecordRxError(UartMonitor_t *monitor)
{
    if (monitor != NULL) {
        monitor->rx_error_count++;
    }
}

void UartMonitor_RecordRxOverflow(UartMonitor_t *monitor)
{
    if (monitor != NULL) {
        monitor->rx_overflow_count++;
    }
}

void UartMonitor_RecordTxError(UartMonitor_t *monitor)
{
    if (monitor != NULL) {
        monitor->tx_error_count++;
    }
}

uint16_t UartMonitor_GetLastRxSize(const UartMonitor_t *monitor)
{
    return (monitor != NULL) ? monitor->last_rx_size : 0U;
}

uint8_t UartMonitor_GetLastRxEvent(const UartMonitor_t *monitor)
{
    return (monitor != NULL) ? monitor->last_rx_event : 0U;
}

static uint8_t uart_rx_dma[UART_MAX_INSTANCES][2][UART_DMA_BUFFER_SIZE]
    __attribute__((section(".bss.bsp_uart_dma"), aligned(32)));
static uint8_t uart_tx_dma[UART_MAX_INSTANCES][UART_TX_BUFFER_SIZE]
    __attribute__((section(".bss.bsp_uart_dma"), aligned(32)));

static UartInstance_t *uart_instances[UART_MAX_INSTANCES];
static uint8_t uart_instance_count;

static UartInstance_t *UART_Find(UART_HandleTypeDef *huart)
{
    uint8_t i;

    for (i = 0U; i < uart_instance_count; ++i) {
        if (uart_instances[i] != NULL && uart_instances[i]->huart == huart) {
            return uart_instances[i];
        }
    }
    return NULL;
}

/* 把实例挂到第 index 组内部 DMA 缓冲上 */
static void UART_AttachBuffers(UartInstance_t *uart, uint8_t index)
{
    uart->instance_index = index;
    uart->rx_buffer[0] = uart_rx_dma[index][0];
    uart->rx_buffer[1] = uart_rx_dma[index][1];
    uart->tx_buffer = uart_tx_dma[index];
    uart->rx_capacity = UART_DMA_BUFFER_SIZE;
    uart->active_buffer = 0U;
    uart->registered = 1U;
}

void UART_Register(UartInstance_t *uart)
{
    uint8_t i;
    uint8_t index;

    /* Re-registering an object is harmless and avoids allocating a second
       buffer pair when init code is called more than once. */
    for (i = 0U; i < uart_instance_count; ++i) {
        if (uart_instances[i] == uart ||
            (uart_instances[i] != NULL &&
             uart_instances[i]->huart == uart->huart)) {
            index = uart_instances[i]->instance_index;
            uart_instances[i] = uart;
            UART_AttachBuffers(uart, index);
            return;
        }
    }

    index = UART_MAX_INSTANCES;
    for (i = 0U; i < uart_instance_count; ++i) {
        if (uart_instances[i] == NULL) {
            index = i;
            break;
        }
    }
    if (index == UART_MAX_INSTANCES) {
        if (uart_instance_count >= UART_MAX_INSTANCES) {
            return;
        }
        index = uart_instance_count++;
    }

    uart_instances[index] = uart;
    UART_AttachBuffers(uart, index);
    uart->rx_started = 0U;
    uart->tx_busy = 0U;
    UartMonitor_Reset(&uart->monitor);
}

void UART_Unregister(UartInstance_t *uart)
{
    uint8_t i;

    if (uart == NULL) {
        return;
    }

    for (i = 0U; i < uart_instance_count; ++i) {
        if (uart_instances[i] == uart) {
            uart_instances[i] = NULL;
            while (uart_instance_count != 0U &&
                   uart_instances[uart_instance_count - 1U] == NULL) {
                uart_instance_count--;
            }
            uart->registered = 0U;
            uart->rx_started = 0U;
            uart->tx_busy = 0U;
            return;
        }
    }
}

BaseType_t UART_BindRx(UartInstance_t *uart, UartRxCallback_t on_receive,
                       UartErrorCallback_t on_error, void *context)
{
    if (uart == NULL || on_receive == NULL || uart->on_receive != NULL) {
        return pdFAIL;
    }

    uart->on_receive = on_receive;
    uart->on_error = on_error;
    uart->context = context;
    return pdPASS;
}

HAL_StatusTypeDef UART_StartReceiveEx(UartInstance_t *uart)
{
    HAL_StatusTypeDef status;
    uint8_t *buffer;

    if (uart == NULL || uart->registered == 0U || uart->huart == NULL ||
        uart->rx_capacity == 0U || uart->rx_buffer[uart->active_buffer] == NULL ||
        uart->huart->hdmarx == NULL) {
        return HAL_ERROR;
    }

    buffer = uart->rx_buffer[uart->active_buffer];
    SCB_InvalidateDCache_by_Addr(buffer, (int32_t)uart->rx_capacity);
    status = HAL_UARTEx_ReceiveToIdle_DMA(uart->huart, buffer,
                                          uart->rx_capacity);
    if (status == HAL_OK) {
        /* Half-transfer callbacks are not packet boundaries.  Keeping only
           IDLE and transfer-complete events matches serialleg2027. */
        __HAL_DMA_DISABLE_IT(uart->huart->hdmarx, DMA_IT_HT);
        uart->rx_started = 1U;
    } else {
        uart->rx_started = 0U;
    }
    return status;
}

void UART_StartReceive(UartInstance_t *uart)
{
    (void)UART_StartReceiveEx(uart);
}

HAL_StatusTypeDef UART_StopReceive(UartInstance_t *uart)
{
    HAL_StatusTypeDef status;

    if (uart == NULL || uart->registered == 0U || uart->huart == NULL) {
        return HAL_ERROR;
    }
    if (uart->rx_started == 0U) {
        return HAL_OK;
    }
    status = HAL_UART_AbortReceive(uart->huart);
    if (status == HAL_OK) {
        uart->rx_started = 0U;
    }
    return status;
}

HAL_StatusTypeDef UART_TransmitDMAEx(UartInstance_t *uart,
                                     const uint8_t *data,
                                     uint16_t size)
{
    HAL_StatusTypeDef status;

    if (uart == NULL || uart->registered == 0U || uart->huart == NULL ||
        uart->tx_buffer == NULL || data == NULL || size == 0U ||
        size > UART_TX_BUFFER_SIZE) {
        return HAL_ERROR;
    }
    if (uart->tx_busy != 0U || uart->huart->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }

    memcpy(uart->tx_buffer, data, size);
    SCB_CleanDCache_by_Addr((uint32_t *)uart->tx_buffer, (int32_t)size);
    status = HAL_UART_Transmit_DMA(uart->huart, uart->tx_buffer, size);
    if (status == HAL_OK) {
        uart->tx_busy = 1U;
    }
    return status;
}

void UART_TransmitDMA(UartInstance_t *uart, const uint8_t *data,
                      uint16_t size)
{
    (void)UART_TransmitDMAEx(uart, data, size);
}

uint16_t UART_GetLastRxSize(const UartInstance_t *uart)
{
    return (uart != NULL) ? UartMonitor_GetLastRxSize(&uart->monitor) : 0U;
}

uint8_t UART_GetLastRxEvent(const UartInstance_t *uart)
{
    return (uart != NULL) ? UartMonitor_GetLastRxEvent(&uart->monitor) : 0U;
}

void UART_ResetStats(UartInstance_t *uart)
{
    if (uart == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    UartMonitor_Reset(&uart->monitor);
    taskEXIT_CRITICAL();
}

/* 先切换缓冲并重启 DMA，再解包：重启越早，两包之间漏字节的窗口越短
 * 两块缓冲交替，刚收完的那块在回调返回前不会被 DMA 再写 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    UartInstance_t *uart = UART_Find(huart);
    uint8_t *completed;

    if (uart == NULL || uart->registered == 0U) {
        return;
    }

    completed = uart->rx_buffer[uart->active_buffer];
    UartMonitor_RecordRx(&uart->monitor, size,
                         (huart != NULL) ? (uint8_t)huart->RxEventType : 0U);
    uart->rx_started = 0U;

    uart->active_buffer ^= 1U;
    if (UART_StartReceiveEx(uart) != HAL_OK) {
        UartMonitor_RecordRxError(&uart->monitor);
    }

    if (size == 0U) {
        UartMonitor_RecordRxError(&uart->monitor);
    } else if (size > uart->rx_capacity) {
        UartMonitor_RecordRxOverflow(&uart->monitor);
    } else if (uart->on_receive != NULL) {
        SCB_InvalidateDCache_by_Addr(completed, (int32_t)size);
        uart->on_receive(uart->context, completed, size);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UartInstance_t *uart = UART_Find(huart);

    if (uart != NULL) {
        uart->tx_busy = 0U;
    }
}

/* HAL 在 DMA 接收出错（溢出、帧错、噪声）时会中止传输并回到 READY，不会自己重新开始
 * 不重启的话这个口从此收不到数据，所以只要接收应该在跑、而 HAL 已经停了，就立刻重启
 * 发送侧的错误不影响接收，RxState 仍是 BUSY_RX，此时不动接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UartInstance_t *uart = UART_Find(huart);

    if (uart == NULL) {
        return;
    }

    if (huart->gState != HAL_UART_STATE_BUSY_TX) {
        uart->tx_busy = 0U;
    }

    if (uart->rx_started != 0U && huart->RxState == HAL_UART_STATE_READY) {
        UartMonitor_RecordRxError(&uart->monitor);
        if (uart->on_error != NULL) {
            uart->on_error(uart->context);
        }
        uart->rx_started = 0U;
        if (UART_StartReceiveEx(uart) != HAL_OK) {
            UartMonitor_RecordRxError(&uart->monitor);
        }
    } else {
        UartMonitor_RecordTxError(&uart->monitor);
    }
}

#include "bsp_uart.h"

#include <stddef.h>
#include <string.h>

/* DMA buffers live in a dedicated section so the linker can place them in a
   DMA-visible RAM region.  The section name is kept compatible with the
   previous BSP implementation. */
static uint8_t uart_rx_dma[BSP_UART_MAX_INSTANCES][2][BSP_UART_DMA_BUFFER_SIZE]
    __attribute__((section(".bss.bsp_uart_dma"), aligned(32)));
static uint8_t uart_rx_packets[BSP_UART_MAX_INSTANCES][BSP_UART_RX_SLOT_COUNT]
                             [BSP_UART_DMA_BUFFER_SIZE]
    __attribute__((section(".bss.bsp_uart_dma"), aligned(32)));
static uint16_t uart_rx_packet_size[BSP_UART_MAX_INSTANCES]
                                    [BSP_UART_RX_SLOT_COUNT];
static uint8_t uart_tx_dma[BSP_UART_MAX_INSTANCES][BSP_UART_TX_BUFFER_SIZE]
    __attribute__((section(".bss.bsp_uart_dma"), aligned(32)));

static BSP_UART_t *uart_instances[BSP_UART_MAX_INSTANCES];
static uint8_t uart_instance_count;

/* A shared event group follows the pattern used by serialleg2027.  A task can
   either wait on this group (when the FreeRTOS timer/deferred-call feature is
   enabled) or use BSP_UART_WaitEvents(), which also supports task notices. */
EventGroupHandle_t BSP_UART_EventGroup;

static BSP_UART_t *BSP_UART_Find(UART_HandleTypeDef *huart)
{
    uint8_t i;

    if (huart == NULL) {
        return NULL;
    }

    for (i = 0U; i < uart_instance_count; ++i) {
        if (uart_instances[i] != NULL && uart_instances[i]->huart == huart) {
            return uart_instances[i];
        }
    }
    return NULL;
}

static uint8_t BSP_UART_InISR(void)
{
    return (__get_IPSR() != 0U) ? 1U : 0U;
}

static EventBits_t BSP_UART_EventGroupBits(const BSP_UART_t *uart,
                                           EventBits_t bits)
{
    EventBits_t mapped = bits;

    if (uart == NULL) {
        return bits;
    }

    if ((bits & BSP_UART_EVENT_RX_READY) != 0U && uart->rx_event_bit != 0U) {
        mapped |= uart->rx_event_bit;
    }
    if ((bits & (BSP_UART_EVENT_RX_OVERFLOW | BSP_UART_EVENT_RX_ERROR |
                 BSP_UART_EVENT_TX_ERROR)) != 0U &&
        uart->error_event_bit != 0U) {
        mapped |= uart->error_event_bit;
    }
    return mapped;
}

/* Record an event in a cache-independent word, then wake a task.  The
   notification path is always available in this project.  Event-group ISR
   setting requires either configUSE_TRACE_FACILITY, configUSE_TIMERS, or
   INCLUDE_xTimerPendFunctionCall; otherwise FreeRTOS has no deferred-call
   implementation and calling xEventGroupSetBitsFromISR would not link. */
static EventBits_t BSP_UART_RecordEvents(BSP_UART_t *uart, EventBits_t bits)
{
    EventBits_t group_bits;

    if (uart == NULL || bits == 0U) {
        return 0U;
    }

    uart->pending_events |= bits;
    group_bits = BSP_UART_EventGroupBits(uart, bits);

    if (BSP_UART_InISR() != 0U) {
        BaseType_t higher_priority_task_woken = pdFALSE;

#if (configUSE_TRACE_FACILITY == 1) || (configUSE_TIMERS == 1) || \
    (INCLUDE_xTimerPendFunctionCall == 1)
        if (uart->event_group != NULL) {
            (void)xEventGroupSetBitsFromISR(uart->event_group, group_bits,
                                            &higher_priority_task_woken);
        }
#endif
        if (uart->notify_task != NULL) {
            (void)xTaskNotifyFromISR(uart->notify_task, (uint32_t)bits,
                                     eSetBits, &higher_priority_task_woken);
        }
        portYIELD_FROM_ISR(higher_priority_task_woken);
    } else {
        if (uart->event_group != NULL) {
            (void)xEventGroupSetBits(uart->event_group, group_bits);
        }
        if (uart->notify_task != NULL) {
            (void)xTaskNotify(uart->notify_task, (uint32_t)bits, eSetBits);
        }
    }

    return bits;
}

static void BSP_UART_QueueReset(BSP_UART_t *uart)
{
    if (uart == NULL) {
        return;
    }

    uart->rx_read_index = 0U;
    uart->rx_write_index = 0U;
    uart->rx_pending_count = 0U;
}

/* Reserve a packet slot before copying.  The interrupt and task paths use the
   matching FreeRTOS critical-section primitive so a consumer never sees a
   partially written packet. */
static BaseType_t BSP_UART_QueuePush(BSP_UART_t *uart,
                                     const uint8_t *data, uint16_t size)
{
    uint8_t index;
    BaseType_t accepted = pdFALSE;

    if (uart == NULL || data == NULL || size == 0U ||
        size > BSP_UART_DMA_BUFFER_SIZE) {
        return pdFALSE;
    }

    if (BSP_UART_InISR() != 0U) {
        UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
        if (uart->rx_pending_count < BSP_UART_RX_SLOT_COUNT) {
            index = uart->rx_write_index;
            uart->rx_write_index = (uint8_t)((index + 1U) %
                                              BSP_UART_RX_SLOT_COUNT);
            uart->rx_pending_count++;
            accepted = pdTRUE;
        }
        taskEXIT_CRITICAL_FROM_ISR(saved);
    } else {
        taskENTER_CRITICAL();
        if (uart->rx_pending_count < BSP_UART_RX_SLOT_COUNT) {
            index = uart->rx_write_index;
            uart->rx_write_index = (uint8_t)((index + 1U) %
                                              BSP_UART_RX_SLOT_COUNT);
            uart->rx_pending_count++;
            accepted = pdTRUE;
        }
        taskEXIT_CRITICAL();
    }

    if (accepted == pdFALSE) {
        return pdFALSE;
    }

    memcpy(uart_rx_packets[uart->instance_index][index], data, size);
    uart_rx_packet_size[uart->instance_index][index] = size;
    return pdTRUE;
}

static BaseType_t BSP_UART_QueuePop(BSP_UART_t *uart, uint8_t *data,
                                    uint16_t capacity, uint16_t *size)
{
    uint8_t index;
    uint16_t packet_size;
    BaseType_t accepted = pdFALSE;

    if (uart == NULL || data == NULL || size == NULL || capacity == 0U) {
        return pdFALSE;
    }

    taskENTER_CRITICAL();
    if (uart->rx_pending_count != 0U) {
        index = uart->rx_read_index;
        packet_size = uart_rx_packet_size[uart->instance_index][index];
        if (packet_size <= capacity) {
            memcpy(data, uart_rx_packets[uart->instance_index][index],
                   packet_size);
            *size = packet_size;
            uart->rx_read_index = (uint8_t)((index + 1U) %
                                             BSP_UART_RX_SLOT_COUNT);
            uart->rx_pending_count--;
            accepted = pdTRUE;
        }
    }
    taskEXIT_CRITICAL();

    return accepted;
}

void BSP_UART_Register(BSP_UART_t *uart)
{
    uint8_t i;
    uint8_t index;

    if (uart == NULL || uart->huart == NULL) {
        return;
    }

    /* Re-registering an object is harmless and avoids allocating a second
       buffer pair when init code is called more than once. */
    for (i = 0U; i < uart_instance_count; ++i) {
        if (uart_instances[i] == uart ||
            (uart_instances[i] != NULL &&
             uart_instances[i]->huart == uart->huart)) {
            uart->instance_index = uart_instances[i]->instance_index;
            uart_instances[i] = uart;
            uart->rx_buffer[0] = uart_rx_dma[uart->instance_index][0];
            uart->rx_buffer[1] = uart_rx_dma[uart->instance_index][1];
            uart->tx_buffer = uart_tx_dma[uart->instance_index];
            uart->rx_capacity = BSP_UART_DMA_BUFFER_SIZE;
            uart->active_buffer = 0U;
            uart->registered = 1U;
            if (uart->rx_event_bit == 0U) {
                uart->rx_event_bit = BSP_UART_EVENT_RX_READY;
            }
            if (uart->error_event_bit == 0U) {
                uart->error_event_bit = BSP_UART_EVENT_RX_ERROR;
            }
            if (BSP_UART_EventGroup == NULL) {
                BSP_UART_EventGroup = xEventGroupCreate();
            }
            if (uart->event_group == NULL) {
                uart->event_group = BSP_UART_EventGroup;
            }
            BSP_UART_QueueReset(uart);
            return;
        }
    }

    index = BSP_UART_MAX_INSTANCES;
    for (i = 0U; i < uart_instance_count; ++i) {
        if (uart_instances[i] == NULL) {
            index = i;
            break;
        }
    }
    if (index == BSP_UART_MAX_INSTANCES) {
        if (uart_instance_count >= BSP_UART_MAX_INSTANCES) {
            return;
        }
        index = uart_instance_count++;
    }

    if (index >= BSP_UART_MAX_INSTANCES) {
        return;
    }
    uart_instances[index] = uart;
    uart->instance_index = index;
    uart->rx_buffer[0] = uart_rx_dma[index][0];
    uart->rx_buffer[1] = uart_rx_dma[index][1];
    uart->tx_buffer = uart_tx_dma[index];
    uart->rx_capacity = BSP_UART_DMA_BUFFER_SIZE;
    uart->active_buffer = 0U;
    uart->rx_started = 0U;
    uart->tx_busy = 0U;
    uart->registered = 1U;
    uart->rx_event_bit = (uart->rx_event_bit != 0U) ? uart->rx_event_bit :
                         BSP_UART_EVENT_RX_READY;
    uart->error_event_bit = (uart->error_event_bit != 0U) ?
                            uart->error_event_bit : BSP_UART_EVENT_RX_ERROR;
    if (BSP_UART_EventGroup == NULL) {
        BSP_UART_EventGroup = xEventGroupCreate();
    }
    if (uart->event_group == NULL) {
        uart->event_group = BSP_UART_EventGroup;
    }
    uart->pending_events = 0U;
    uart->last_rx_size = 0U;
    uart->last_rx_event = 0U;
    BSP_UART_QueueReset(uart);
}

void BSP_UART_Unregister(BSP_UART_t *uart)
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

HAL_StatusTypeDef BSP_UART_StartReceiveEx(BSP_UART_t *uart)
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

void BSP_UART_StartReceive(BSP_UART_t *uart)
{
    (void)BSP_UART_StartReceiveEx(uart);
}

HAL_StatusTypeDef BSP_UART_StopReceive(BSP_UART_t *uart)
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

HAL_StatusTypeDef BSP_UART_TransmitDMAEx(BSP_UART_t *uart,
                                         const uint8_t *data,
                                         uint16_t size)
{
    HAL_StatusTypeDef status;

    if (uart == NULL || uart->registered == 0U || uart->huart == NULL ||
        uart->tx_buffer == NULL || data == NULL || size == 0U ||
        size > BSP_UART_TX_BUFFER_SIZE) {
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

void BSP_UART_TransmitDMA(BSP_UART_t *uart, const uint8_t *data,
                          uint16_t size)
{
    (void)BSP_UART_TransmitDMAEx(uart, data, size);
}

BaseType_t BSP_UART_RxPop(BSP_UART_t *uart, uint8_t *data,
                          uint16_t capacity, uint16_t *size)
{
    if (size != NULL) {
        *size = 0U;
    }
    return BSP_UART_QueuePop(uart, data, capacity, size);
}

BaseType_t BSP_UART_Receive(BSP_UART_t *uart, uint8_t *data,
                            uint16_t capacity, uint16_t *size,
                            TickType_t timeout)
{
    TickType_t start_tick;
    TickType_t elapsed;
    TickType_t remaining;

    if (BSP_UART_RxPop(uart, data, capacity, size) == pdTRUE) {
        return pdTRUE;
    }
    if (timeout == 0U || uart == NULL) {
        return pdFALSE;
    }

    start_tick = xTaskGetTickCount();
    remaining = timeout;
    do {
        (void)BSP_UART_WaitEvents(uart,
                                   BSP_UART_EVENT_RX_READY |
                                   BSP_UART_EVENT_RX_OVERFLOW |
                                   BSP_UART_EVENT_RX_ERROR,
                                   remaining);
        if (BSP_UART_RxPop(uart, data, capacity, size) == pdTRUE) {
            return pdTRUE;
        }
        elapsed = xTaskGetTickCount() - start_tick;
        if (elapsed >= timeout) {
            break;
        }
        remaining = timeout - elapsed;
    } while (remaining != 0U);

    return pdFALSE;
}

BaseType_t BSP_UART_Read(BSP_UART_t *uart, uint8_t *data,
                         uint16_t capacity, uint16_t *size)
{
    return BSP_UART_RxPop(uart, data, capacity, size);
}

uint16_t BSP_UART_RxAvailable(const BSP_UART_t *uart)
{
    return (uart != NULL) ? (uint16_t)uart->rx_pending_count : 0U;
}

void BSP_UART_SetEventGroup(BSP_UART_t *uart, EventGroupHandle_t group,
                            EventBits_t rx_bit, EventBits_t error_bit)
{
    if (uart == NULL) {
        return;
    }
    if (group == NULL) {
        if (BSP_UART_EventGroup == NULL) {
            BSP_UART_EventGroup = xEventGroupCreate();
        }
        group = BSP_UART_EventGroup;
    }
    uart->event_group = group;
    uart->rx_event_bit = (rx_bit != 0U) ? rx_bit : BSP_UART_EVENT_RX_READY;
    uart->error_event_bit = (error_bit != 0U) ? error_bit :
                            BSP_UART_EVENT_RX_ERROR;
}

EventGroupHandle_t BSP_UART_GetEventGroup(const BSP_UART_t *uart)
{
    return (uart != NULL) ? uart->event_group : NULL;
}

BaseType_t BSP_UART_SetNotifyTask(BSP_UART_t *uart, TaskHandle_t task)
{
    if (uart == NULL) {
        return pdFALSE;
    }
    uart->notify_task = task;
    return pdTRUE;
}

EventBits_t BSP_UART_WaitEvents(BSP_UART_t *uart, EventBits_t bits,
                                TickType_t timeout)
{
    EventBits_t pending;
    uint32_t notification_value = 0U;
    TaskHandle_t current_task;

    if (uart == NULL || bits == 0U) {
        return 0U;
    }

    pending = BSP_UART_GetEvents(uart, pdTRUE);
    if ((pending & bits) != 0U) {
        return pending;
    }

    current_task = xTaskGetCurrentTaskHandle();
    if (uart->notify_task == NULL) {
        uart->notify_task = current_task;
    }

    if (uart->notify_task == current_task && current_task != NULL) {
        (void)xTaskNotifyWait(0U, (uint32_t)bits, &notification_value,
                              timeout);
        pending |= (EventBits_t)notification_value;
        pending |= BSP_UART_GetEvents(uart, pdTRUE);
        return pending;
    }

    if (uart->event_group != NULL) {
        pending |= xEventGroupWaitBits(uart->event_group,
                                       BSP_UART_EventGroupBits(uart, bits),
                                       pdTRUE, pdFALSE, timeout);
    }
    return pending;
}

EventBits_t BSP_UART_GetEvents(BSP_UART_t *uart, BaseType_t clear)
{
    EventBits_t events;

    if (uart == NULL) {
        return 0U;
    }

    taskENTER_CRITICAL();
    events = uart->pending_events;
    if (clear != pdFALSE) {
        uart->pending_events = 0U;
    }
    taskEXIT_CRITICAL();
    return events;
}

EventBits_t BSP_UART_SetEvents(BSP_UART_t *uart, EventBits_t bits)
{
    return BSP_UART_RecordEvents(uart, bits);
}

uint16_t BSP_UART_GetLastRxSize(const BSP_UART_t *uart)
{
    return (uart != NULL) ? uart->last_rx_size : 0U;
}

uint8_t BSP_UART_GetLastRxEvent(const BSP_UART_t *uart)
{
    return (uart != NULL) ? uart->last_rx_event : 0U;
}

void BSP_UART_ResetStats(BSP_UART_t *uart)
{
    if (uart == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    uart->rx_event_count = 0U;
    uart->rx_error_count = 0U;
    uart->rx_overflow_count = 0U;
    uart->tx_error_count = 0U;
    uart->last_rx_size = 0U;
    uart->last_rx_event = 0U;
    uart->pending_events = 0U;
    taskEXIT_CRITICAL();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    BSP_UART_t *uart = BSP_UART_Find(huart);
    uint8_t completed_index;
    uint8_t *completed;

    if (uart == NULL || uart->registered == 0U) {
        return;
    }

    completed_index = uart->active_buffer;
    completed = uart->rx_buffer[completed_index];
    uart->last_rx_size = size;
    uart->last_rx_event = (huart != NULL) ? (uint8_t)huart->RxEventType : 0U;
    uart->rx_event_count++;
    uart->rx_started = 0U;

    if (size != 0U && size <= uart->rx_capacity && completed != NULL) {
        SCB_InvalidateDCache_by_Addr(completed, (int32_t)size);
        if (BSP_UART_QueuePush(uart, completed, size) == pdTRUE) {
            (void)BSP_UART_RecordEvents(uart, BSP_UART_EVENT_RX_READY);
            if (uart->on_receive != NULL) {
                /* The callback runs in the UART ISR.  Keep it short; tasks
                   should consume packets through BSP_UART_RxPop/Receive. */
                uart->on_receive(uart->context, completed, size);
            }
        } else {
            uart->rx_overflow_count++;
            (void)BSP_UART_RecordEvents(uart, BSP_UART_EVENT_RX_OVERFLOW);
        }
    } else {
        uart->rx_error_count++;
        (void)BSP_UART_RecordEvents(uart, BSP_UART_EVENT_RX_ERROR);
    }

    /* Switch only after the completed buffer has been copied. */
    uart->active_buffer ^= 1U;
    if (BSP_UART_StartReceiveEx(uart) != HAL_OK) {
        uart->rx_error_count++;
        (void)BSP_UART_RecordEvents(uart, BSP_UART_EVENT_RX_ERROR);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_t *uart = BSP_UART_Find(huart);

    if (uart == NULL) {
        return;
    }
    uart->tx_busy = 0U;
    (void)BSP_UART_RecordEvents(uart, BSP_UART_EVENT_TX_DONE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_t *uart = BSP_UART_Find(huart);

    if (uart == NULL) {
        return;
    }

    uart->tx_busy = 0U;
    uart->rx_error_count++;
    (void)BSP_UART_RecordEvents(uart, BSP_UART_EVENT_RX_ERROR |
                                      BSP_UART_EVENT_TX_ERROR);
    if (uart->rx_started == 0U) {
        uart->active_buffer &= 1U;
        (void)BSP_UART_StartReceiveEx(uart);
    }
}

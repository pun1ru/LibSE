#ifndef BSP_UART_H
#define BSP_UART_H

#include "usart.h"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The DMA buffers are deliberately cache-line aligned for STM32H7. */
#define BSP_UART_DMA_BUFFER_SIZE       256U
#define BSP_UART_RX_SLOT_COUNT         4U
#define BSP_UART_TX_BUFFER_SIZE        BSP_UART_DMA_BUFFER_SIZE
#define BSP_UART_MAX_INSTANCES         6U

/* Bits reported by BSP_UART_WaitEvents()/BSP_UART_GetEvents(). */
#define BSP_UART_EVENT_RX_READY        ((EventBits_t)(1UL << 0U))
#define BSP_UART_EVENT_RX_OVERFLOW     ((EventBits_t)(1UL << 1U))
#define BSP_UART_EVENT_RX_ERROR        ((EventBits_t)(1UL << 2U))
#define BSP_UART_EVENT_TX_DONE         ((EventBits_t)(1UL << 3U))
#define BSP_UART_EVENT_TX_ERROR        ((EventBits_t)(1UL << 4U))
#define BSP_UART_EVENT_ALL             ((EventBits_t)(BSP_UART_EVENT_RX_READY | \
                                                       BSP_UART_EVENT_RX_OVERFLOW | \
                                                       BSP_UART_EVENT_RX_ERROR | \
                                                       BSP_UART_EVENT_TX_DONE | \
                                                       BSP_UART_EVENT_TX_ERROR))

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

    /* Optional RTOS notification targets.  Register() creates event_group
       when the caller has not supplied one. */
    EventGroupHandle_t event_group;
    EventBits_t rx_event_bit;
    EventBits_t error_event_bit;
    TaskHandle_t notify_task;

    /* State which is updated by the HAL callback and consumed by a task. */
    volatile EventBits_t pending_events;
    volatile uint16_t last_rx_size;
    volatile uint8_t last_rx_event;
    volatile uint8_t rx_started;
    volatile uint8_t tx_busy;
    volatile uint8_t registered;
    uint8_t instance_index;

    volatile uint8_t rx_read_index;
    volatile uint8_t rx_write_index;
    volatile uint8_t rx_pending_count;
    volatile uint32_t rx_event_count;
    volatile uint32_t rx_error_count;
    volatile uint32_t rx_overflow_count;
    volatile uint32_t tx_error_count;
} BSP_UART_t;

/* Shared receive event group for task layers that wait on several UARTs. */
extern EventGroupHandle_t BSP_UART_EventGroup;

/* Registration assigns cache-safe internal DMA and packet buffers. */
void BSP_UART_Register(BSP_UART_t *uart);
void BSP_UART_Unregister(BSP_UART_t *uart);

/* ReceiveToIdle DMA control.  The legacy void entry point remains available. */
HAL_StatusTypeDef BSP_UART_StartReceiveEx(BSP_UART_t *uart);
void BSP_UART_StartReceive(BSP_UART_t *uart);
HAL_StatusTypeDef BSP_UART_StopReceive(BSP_UART_t *uart);

/* DMA transmit.  The Ex variant returns the HAL status. */
HAL_StatusTypeDef BSP_UART_TransmitDMAEx(BSP_UART_t *uart,
                                         const uint8_t *data,
                                         uint16_t size);
void BSP_UART_TransmitDMA(BSP_UART_t *uart, const uint8_t *data,
                          uint16_t size);

/* Packet-ring consumption.  RxPop is non-blocking; Receive waits for an RX
   event when timeout is non-zero.  A packet is copied into caller storage. */
BaseType_t BSP_UART_RxPop(BSP_UART_t *uart, uint8_t *data,
                          uint16_t capacity, uint16_t *size);
BaseType_t BSP_UART_Receive(BSP_UART_t *uart, uint8_t *data,
                            uint16_t capacity, uint16_t *size,
                            TickType_t timeout);
BaseType_t BSP_UART_Read(BSP_UART_t *uart, uint8_t *data,
                         uint16_t capacity, uint16_t *size);
uint16_t BSP_UART_RxAvailable(const BSP_UART_t *uart);

/* Event-group and task-notification integration.  SetNotifyTask() is useful
   when one task waits for several UARTs; otherwise Receive() binds its first
   wait to the calling task automatically. */
void BSP_UART_SetEventGroup(BSP_UART_t *uart, EventGroupHandle_t group,
                            EventBits_t rx_bit, EventBits_t error_bit);
EventGroupHandle_t BSP_UART_GetEventGroup(const BSP_UART_t *uart);
BaseType_t BSP_UART_SetNotifyTask(BSP_UART_t *uart, TaskHandle_t task);
EventBits_t BSP_UART_WaitEvents(BSP_UART_t *uart, EventBits_t bits,
                                TickType_t timeout);
EventBits_t BSP_UART_GetEvents(BSP_UART_t *uart, BaseType_t clear);
EventBits_t BSP_UART_SetEvents(BSP_UART_t *uart, EventBits_t bits);

/* Optional diagnostics for monitor tasks. */
uint16_t BSP_UART_GetLastRxSize(const BSP_UART_t *uart);
uint8_t BSP_UART_GetLastRxEvent(const BSP_UART_t *uart);
void BSP_UART_ResetStats(BSP_UART_t *uart);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */

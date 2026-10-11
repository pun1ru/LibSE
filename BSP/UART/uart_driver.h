#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "usart.h"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_DMA_BUFFER_SIZE       256U
#define UART_RX_SLOT_COUNT         4U
#define UART_TX_BUFFER_SIZE        UART_DMA_BUFFER_SIZE
#define UART_MAX_INSTANCES         6U

#define UART_EVENT_RX_READY        ((EventBits_t)(1UL << 0U))
#define UART_EVENT_RX_OVERFLOW     ((EventBits_t)(1UL << 1U))
#define UART_EVENT_RX_ERROR        ((EventBits_t)(1UL << 2U))
#define UART_EVENT_TX_DONE         ((EventBits_t)(1UL << 3U))
#define UART_EVENT_TX_ERROR        ((EventBits_t)(1UL << 4U))
#define UART_EVENT_ALL             ((EventBits_t)(UART_EVENT_RX_READY | \
                                                       UART_EVENT_RX_OVERFLOW | \
                                                       UART_EVENT_RX_ERROR | \
                                                       UART_EVENT_TX_DONE | \
                                                       UART_EVENT_TX_ERROR))

typedef struct {
    volatile uint16_t last_rx_size;
    volatile uint8_t last_rx_event;
    volatile uint32_t rx_event_count;
    volatile uint32_t rx_error_count;
    volatile uint32_t rx_overflow_count;
    volatile uint32_t tx_error_count;
} UartMonitor_t;

void UartMonitor_Reset(UartMonitor_t *monitor);
void UartMonitor_RecordRx(UartMonitor_t *monitor, uint16_t size,
                          uint8_t event);
void UartMonitor_RecordRxError(UartMonitor_t *monitor);
void UartMonitor_RecordRxOverflow(UartMonitor_t *monitor);
void UartMonitor_RecordTxError(UartMonitor_t *monitor);
uint16_t UartMonitor_GetLastRxSize(const UartMonitor_t *monitor);
uint8_t UartMonitor_GetLastRxEvent(const UartMonitor_t *monitor);

typedef void (*UartRxCallback_t)(void *context, const uint8_t *data,
                                      uint16_t size);
typedef void (*UartPacketCallback_t)(const uint8_t *data, uint16_t size);
typedef void (*UartErrorCallback_t)(void);

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t *rx_buffer[2];//双缓冲区
    uint8_t *tx_buffer;
    uint16_t rx_capacity;
    uint8_t active_buffer;
    UartRxCallback_t on_receive;//函数指针
    UartPacketCallback_t on_packet;
    UartErrorCallback_t on_error;
    void *context;

    EventGroupHandle_t event_group;//事件组句柄
    EventBits_t rx_event_bit;//事件位
    EventBits_t error_event_bit;
    TaskHandle_t notify_task;//通知任务

    //缓冲区状态位
    volatile EventBits_t pending_events;
    volatile uint8_t rx_started;
    volatile uint8_t tx_busy;
    volatile uint8_t registered;
    uint8_t instance_index;

    volatile uint8_t rx_read_index;
    volatile uint8_t rx_write_index;
    volatile uint8_t rx_pending_count;
    UartMonitor_t monitor;
} UartInstance_t;

/* Shared receive event group for task layers that wait on several UARTs. */
extern EventGroupHandle_t UART_EventGroup;

/* Registration assigns cache-safe internal DMA and packet buffers. */
void UART_Register(UartInstance_t *uart);
void UART_Unregister(UartInstance_t *uart);

/* ReceiveToIdle DMA control.  The legacy void entry point remains available. */
HAL_StatusTypeDef UART_StartReceiveEx(UartInstance_t *uart);
void UART_StartReceive(UartInstance_t *uart);
HAL_StatusTypeDef UART_StopReceive(UartInstance_t *uart);

/* DMA transmit.  The Ex variant returns the HAL status. */
HAL_StatusTypeDef UART_TransmitDMAEx(UartInstance_t *uart,
                                         const uint8_t *data,
                                         uint16_t size);
void UART_TransmitDMA(UartInstance_t *uart, const uint8_t *data,
                          uint16_t size);

/* Packet-ring consumption.  RxPop is non-blocking; Receive waits for an RX
   event when timeout is non-zero.  A packet is copied into caller storage. */
BaseType_t UART_RxPop(UartInstance_t *uart, uint8_t *data,
                          uint16_t capacity, uint16_t *size);
BaseType_t UART_Receive(UartInstance_t *uart, uint8_t *data,
                            uint16_t capacity, uint16_t *size,
                            TickType_t timeout);
BaseType_t UART_Read(UartInstance_t *uart, uint8_t *data,
                         uint16_t capacity, uint16_t *size);
uint16_t UART_RxAvailable(const UartInstance_t *uart);

/* Event-group and task-notification integration.  SetNotifyTask() is useful
   when one task waits for several UARTs; otherwise Receive() binds its first
   wait to the calling task automatically. */
void UART_SetEventGroup(UartInstance_t *uart, EventGroupHandle_t group,
                            EventBits_t rx_bit, EventBits_t error_bit);
EventGroupHandle_t UART_GetEventGroup(const UartInstance_t *uart);
BaseType_t UART_SetNotifyTask(UartInstance_t *uart, TaskHandle_t task);
EventBits_t UART_WaitEvents(UartInstance_t *uart, EventBits_t bits,
                                TickType_t timeout);
EventBits_t UART_GetEvents(UartInstance_t *uart, BaseType_t clear);
EventBits_t UART_SetEvents(UartInstance_t *uart, EventBits_t bits);

/* Optional diagnostics for monitor tasks. */
uint16_t UART_GetLastRxSize(const UartInstance_t *uart);
uint8_t UART_GetLastRxEvent(const UartInstance_t *uart);
void UART_ResetStats(UartInstance_t *uart);

#ifdef __cplusplus
}
#endif

#endif /* UART_DRIVER_H */

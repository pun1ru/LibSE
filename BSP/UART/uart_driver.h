#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 接收：ReceiveToIdle + DMA 双缓冲，参考 damiao_mc02_bsp 的 drv_uart
 * 一包结束（IDLE 或缓冲收满）时，中断回调里先切换缓冲并重启 DMA，再对刚收完的那块调用绑定的 on_receive
 * 解析直接在中断里完成，不经队列和任务，接收绑定的用法同 CAN_BindRx */
#define UART_DMA_BUFFER_SIZE       256U
#define UART_TX_BUFFER_SIZE        UART_DMA_BUFFER_SIZE
#define UART_MAX_INSTANCES         6U

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

/* 接收回调：在串口中断里执行，data 只在回调期间有效（下一包会覆盖这块缓冲），要保留的自己拷走
 * 只解析并写自己的缓存，保持简短，禁止调用非 FromISR 的 FreeRTOS 接口 */
typedef void (*UartRxCallback_t)(void *context, const uint8_t *data,
                                 uint16_t size);
/* 接收出错回调：同在中断里执行，驱动已经重启接收，回调只需清掉解析器的半包状态 */
typedef void (*UartErrorCallback_t)(void *context);

struct UartInstance;
/* 发送回调：由发送任务按周期调用（不是中断），在任务里整理数据，再用 uart 调 UART_TransmitDMAEx 发出去
 * 返回 HAL_BUSY 说明上一帧还在发，回调自己决定丢帧还是下个周期重发 */
typedef void (*UartTxCallback_t)(void *context, struct UartInstance *uart);

typedef struct UartInstance {
    UART_HandleTypeDef *huart;
    uint8_t *rx_buffer[2];      /* 双缓冲区 */
    uint8_t *tx_buffer;
    uint16_t rx_capacity;
    uint8_t active_buffer;      /* DMA 正在写的那块 */
    UartRxCallback_t on_receive;
    UartErrorCallback_t on_error;
    void *rx_context;
    UartTxCallback_t on_transmit;
    void *tx_context;

    volatile uint8_t rx_started;
    volatile uint8_t tx_busy;
    volatile uint8_t registered;
    uint8_t instance_index;
    UartMonitor_t monitor;
} UartInstance_t;

/* Registration assigns cache-safe internal DMA buffers. */
void UART_Register(UartInstance_t *uart);
void UART_Unregister(UartInstance_t *uart);

/* 绑定接收回调，一个串口口只挂一个解析器，已经绑过返回 pdFAIL
 * 应在 UART_Register 之后、UART_StartReceiveEx 之前调用，中断只读不写，不需要加锁 */
BaseType_t UART_BindRx(UartInstance_t *uart, UartRxCallback_t on_receive,
                       UartErrorCallback_t on_error, void *context);

/* 绑定发送回调，一个串口口只挂一个发送方，已经绑过返回 pdFAIL
 * 应在 UART_Register 之后调用；回调由发送任务经 UART_DispatchTx 调用 */
BaseType_t UART_BindTx(UartInstance_t *uart, UartTxCallback_t on_transmit, void *context);
/* 调一次这个口绑定的发送回调，没绑就什么都不做，由发送任务按周期调用 */
void UART_DispatchTx(UartInstance_t *uart);

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

/* Optional diagnostics for monitor tasks. */
uint16_t UART_GetLastRxSize(const UartInstance_t *uart);
uint8_t UART_GetLastRxEvent(const UartInstance_t *uart);
void UART_ResetStats(UartInstance_t *uart);

#ifdef __cplusplus
}
#endif

#endif /* UART_DRIVER_H */

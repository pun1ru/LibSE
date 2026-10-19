#ifndef VOFA_UART_H
#define VOFA_UART_H

#include "uart_driver.h"
#include "vofa_justfloat.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 把拼好的 JustFloat 帧经 UART DMA 发出
 * 返回值沿用 UART_TransmitDMAEx：
 *   HAL_OK     已交给 DMA；帧没有通道时也返回 HAL_OK，什么都不发
 *   HAL_BUSY   上一帧还在发，调用者自己决定丢帧还是下周期重发
 *   HAL_ERROR  参数无效，或帧超过 UART_TX_BUFFER_SIZE
 * 驱动会把数据拷进自己的 DMA 缓冲，返回后帧缓冲可以立刻复用 */
HAL_StatusTypeDef VofaJustFloat_Send(const VofaJustFloat_t *frame, UartInstance_t *uart);

#ifdef __cplusplus
}
#endif

#endif

/**
 * @file WS2812.h
 * @author 3SE
 * @brief WS2812 LED 驱动 — SPI单灯（板载） + PWM+DMA灯带（12灯级联）
 * @version 2.0
 * @date 2025-02-12
 *
 * @note 双接口并存：
 *       - WS2812_SPI_Ctrl()     板载单LED，SPI6/PA07
 *       - WS2812_PWM_xxx()      12灯灯带，TIM2_CH1/PA00（原舵机），DMA+PWM模拟
 *
 * @copyright Copyright (c) 2025
 */

#ifndef __WS2812_H__
#define __WS2812_H__

#include "main.h"
#include "general_define.h"

/*---------------------------------------------------------------------------SPI 板载单灯（PA07, SPI6）-----------------------------------------------------*/
#define WS2812_SPI_UNIT BOARD_LED_SPI
extern SPI_HandleTypeDef WS2812_SPI_UNIT;

void WS2812_SPI_Ctrl(uint8_t r, uint8_t g, uint8_t b);

/*---------------------------------------------------------------------------PWM+DMA 灯带（PA00, TIM2_CH1）------------------------------------------------*/
/** @brief 灯带 LED 数量 */
#define WS2812_NUM_LEDS    12

/** @brief 灯带 PWM 初始化（重新配置 TIM2_CH1 为 800kHz，配置 DMA1_Stream0）
 *  @note 调用后 TIM2 不再输出舵机 PWM，原 servo 代码需注释
 */
void WS2812_PWM_Init(void);

/** @brief 设置单个 LED 颜色（不立即发送，需调 WS2812_Send() 刷新）
 *  @param index LED 编号 0..WS2812_NUM_LEDS-1
 *  @param r     红色亮度 0-255
 *  @param g     绿色亮度 0-255
 *  @param b     蓝色亮度 0-255
 */
void WS2812_SetLED(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

/** @brief 设置全部 LED 同色（不立即发送） */
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);

/** @brief 将缓冲区数据通过 DMA+PWM 发送到灯带（非阻塞，立即返回） */
void WS2812_Send(void);

#endif

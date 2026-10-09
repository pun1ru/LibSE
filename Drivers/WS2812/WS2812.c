/**
 * @file WS2812.c
 * @author 3SE
 * @brief WS2812 LED 驱动 — SPI 板载单灯 + PWM+DMA 12灯灯带
 *
 * === SPI 板载灯（兼容旧代码）===
 *   PA07 / SPI6 驱动板载单颗 WS2812，用作启动状态指示
 *
 * === PWM+DMA 灯带（新增）===
 *   PA00 / TIM2_CH1 → DMA1_Stream0 + DMAMUX1 → 12 灯级联 WS2812 灯带
 *
 *   原理：
 *     TIM2 重配为 800kHz PWM（PSC=0, ARR=299），每个 PWM 周期 = 1.25µs = 1bit
 *     0-码: CCR= 96 → 32%占空比（0.40µs 高 / 0.85µs 低）
 *     1-码: CCR=192 → 64%占空比（0.80µs 高 / 0.45µs 低）
 *     CCR1 预装载使能 → DMA 写入的新值在定时器溢出后才生效，避免半周期跳变
 *     DMA 在每次比较匹配时自动加载下一个 CCR → 硬件波形生成，零 CPU 干预
 *     数据帧后接 50 个 CCR=0（62.5µs 低电平）→ 灯带自动锁存
 *
 *   时序参考（WS2812B Datasheet）：
 *     T0H=0.40µs  T0L=0.85µs  T=1.25µs
 *     T1H=0.80µs  T1L=0.45µs  T=1.25µs
 *     RESET >50µs
 *
 * @version 2.0
 * @date 2025-02-12
 *
 * @copyright Copyright (c) 2025
 */

#include "ws2812.h"
#include "tim.h"                  // htim2 声明
#include "stm32h7xx_hal_dma.h"   // DMA_REQUEST_TIM2_CH1

/*---------------------------------------------------------------------------WS2812 时序常量------------------------------------------------------------------*/
/** @brief TIM2 输入时钟（APB1 Timer Clock） */
#define WS2812_TIM_CLK_HZ       240000000UL

/** @brief 位速率 = 800kHz → 位周期 = 1.25µs */
#define WS2812_BIT_RATE_HZ      800000UL

/** @brief ARR 值（计数器周期 ticks）：clk / bit_rate = 240M / 800k = 300 */
#define WS2812_ARR_VAL          ((WS2812_TIM_CLK_HZ) / (WS2812_BIT_RATE_HZ))

/** @brief ARR 寄存器值 = 300 - 1 = 299 */
#define WS2812_ARR_REG          ((WS2812_ARR_VAL) - 1)

/** @brief 0-码 CCR：300 × 32% = 96（0.40µs 高电平） */
#define WS2812_CCR_0            ((WS2812_ARR_VAL) * 32 / 100)

/** @brief 1-码 CCR：300 × 64% = 192（0.80µs 高电平） */
#define WS2812_CCR_1            ((WS2812_ARR_VAL) * 64 / 100)

/** @brief RESET 位数：50 × 1.25µs = 62.5µs > 50µs */
#define WS2812_RESET_BITS       50

/** @brief 每灯位数：G(8) + R(8) + B(8) = 24 */
#define WS2812_BITS_PER_LED     24

/** @brief DMA 缓冲区总大小：12灯×24bit + 50bit RESET = 338 */
#define WS2812_TOTAL_BITS       (WS2812_NUM_LEDS * WS2812_BITS_PER_LED + WS2812_RESET_BITS)

/*---------------------------------------------------------------------------TIM2 硬件指针------------------------------------------------------------------*/
/** @brief TIM2 外设基址指针 */
#define WS2812_TIM_INSTANCE     TIM2

/*---------------------------------------------------------------------------DMA 配置常量------------------------------------------------------------------*/
/** @brief 使用的 DMA Stream：DMA1_Stream0 */
#define WS2812_DMA_STREAM       DMA1_Stream0

/** @brief DMAMUX1 请求 ID：TIM2_CH1 = 18U（定义于 stm32h7xx_hal_dma.h） */
#define WS2812_DMA_REQUEST      DMA_REQUEST_TIM2_CH1

/** @brief 32-bit 数据宽度组合：MSIZE=word + PSIZE=word */
#define DMA_SxCR_32BIT          (DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1)

/*---------------------------------------------------------------------------全局静态缓冲区------------------------------------------------------------------*/

/** @brief LED 颜色缓冲区 — 12 灯 × GRB */
static struct
{
    uint8_t g;  // 绿（WS2812 顺序：G→R→B）
    uint8_t r;  // 红
    uint8_t b;  // 蓝
} led_color_buf[WS2812_NUM_LEDS];

/** @brief DMA CCR 值缓冲区 — 每个元素对应一个 PWM 周期的 CCR 寄存器值 */
static uint32_t ccr_dma_buf[WS2812_TOTAL_BITS];

/** @brief 初始化完成标志 */
static uint8_t pwm_ready = 0;

/*---------------------------------------------------------------------------SPI 板载单灯（保持兼容）-----------------------------------------------------*/

/** @brief SPI 0-码脉冲编码 */
#define WS2812_LowLevel    0xC0
/** @brief SPI 1-码脉冲编码 */
#define WS2812_HighLevel   0xF0

/**
 * @brief 板载单 LED 控制（SPI6 / PA07）
 * @note  仅控制板载 1 颗 WS2812，与灯带（PA00）独立
 */
void WS2812_SPI_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    for (int i = 0; i < 8; i++)
    {
        txbuf[7 - i]  = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
}

/*---------------------------------------------------------------------------内部辅助函数------------------------------------------------------------------*/

/**
 * @brief 将 LED 颜色缓冲区编码为 CCR DMA 缓冲区
 * @note  级联顺序：LED[0] 先收到 24bit → LED[0] 显示 buffer 中 LED[0] 的颜色
 *        数据帧：每灯 G[7:0] → R[7:0] → B[7:0]，各字节高位先发
 *        RESET 段（最后 50 个 CCR=0）已在 WS2812_PWM_Init 中预填
 */
static void WS2812_EncodeBuffer(void)
{
    uint32_t idx = 0;

    for (int led = 0; led < WS2812_NUM_LEDS; led++)
    {
        uint8_t g = led_color_buf[led].g;
        uint8_t r = led_color_buf[led].r;
        uint8_t b = led_color_buf[led].b;

        /* G — 高位先发 */
        for (int bit = 7; bit >= 0; bit--)
        {
            ccr_dma_buf[idx++] = (g >> bit) & 0x01 ? WS2812_CCR_1 : WS2812_CCR_0;
        }
        /* R — 高位先发 */
        for (int bit = 7; bit >= 0; bit--)
        {
            ccr_dma_buf[idx++] = (r >> bit) & 0x01 ? WS2812_CCR_1 : WS2812_CCR_0;
        }
        /* B — 高位先发 */
        for (int bit = 7; bit >= 0; bit--)
        {
            ccr_dma_buf[idx++] = (b >> bit) & 0x01 ? WS2812_CCR_1 : WS2812_CCR_0;
        }
    }
    /* idx 现在 == WS2812_NUM_LEDS * WS2812_BITS_PER_LED (= 288)
       RESET 段（[288..337]）已在 Init 中预填为 0，无需每次重复赋值 */
}

/*---------------------------------------------------------------------------PWM+DMA 灯带公开 API----------------------------------------------------------*/

/**
 * @brief   初始化 WS2812 灯带 PWM+DMA 驱动
 * @note    - 停止并重配置 TIM2_CH1：50Hz 舵机 → 800kHz WS2812
 *          - 使能 CCR1 预装载 + DMA1_Stream0 自动喂 CCR
 *          - PA00 引脚配置不变（仍为 AF1 TIM2_CH1）
 *          - 调用后原舵机 servo 功能失效，需注释 robot_control_task.c 中
 *            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ...) 相关代码
 */
void WS2812_PWM_Init(void)
{
    /*--- 1. 停止旧舵机 PWM ---*/
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    /*--- 2. 更新 TIM2 时基并通过 HAL 重新初始化 ---*/
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = WS2812_ARR_REG;                   // 299
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim2);                                        // 应用时基到寄存器

    /*--- 3. 配置 PWM 通道（HAL 自动设置 CC1E + PWM1 + 极性）---*/
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;                         // CCR=0 → 空闲低电平
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    /*--- 4. 使能 CCR1 预装载：DMA 写入在溢出时才生效，避免半周期电平跳变 ---*/
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

    /*--- 5. 启动 PWM（CC1E=1 + CEN=1）→ PA0 输出持续低电平, 灯带空闲 ---*/
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    /*--- 6. LED 颜色缓冲区全部清零（默认灭）---*/
    for (int i = 0; i < WS2812_NUM_LEDS; i++)
    {
        led_color_buf[i].r = 0;
        led_color_buf[i].g = 0;
        led_color_buf[i].b = 0;
    }

    /*--- 7. 预填充 DMA 缓冲区的 RESET 段（最后 WS2812_RESET_BITS 个，全部 CCR=0）---*/
    for (int i = 0; i < WS2812_RESET_BITS; i++)
    {
        ccr_dma_buf[WS2812_NUM_LEDS * WS2812_BITS_PER_LED + i] = 0;
    }

    /*--- 8. 使能 DMA1 时钟 + 配置 DMAMUX1_Channel0 ---*/
    __HAL_RCC_DMA1_CLK_ENABLE();
    DMAMUX1_Channel0->CCR = WS2812_DMA_REQUEST;       // 请求源 = TIM2_CH1 (ID=18)

    /*--- 9. 预配置 DMA1_Stream0（发送时动态启停）---*/
    WS2812_DMA_STREAM->CR = 0;
    while (WS2812_DMA_STREAM->CR & DMA_SxCR_EN);

    WS2812_DMA_STREAM->PAR  = (uint32_t)&TIM2->CCR1;  // 外设地址 = CCR1
    WS2812_DMA_STREAM->M0AR = 0;                       // 内存地址 → 发送时再填
    WS2812_DMA_STREAM->NDTR = 0;                       // 传输次数 → 发送时再填
    WS2812_DMA_STREAM->FCR  = 0;                       // 直接模式（无 FIFO）

    /* MINC + DIR_M2P + 32-bit 数据宽度 */
    WS2812_DMA_STREAM->CR = (0x1UL << DMA_SxCR_MINC_Pos)
                          | (0x1UL << DMA_SxCR_DIR_Pos)
                          | DMA_SxCR_32BIT;

    pwm_ready = 1;
}

/**
 * @brief   设置单颗 LED 颜色（不立即发送）
 * @param   index LED 编号 0..11（0 = 最靠近 MCU 的灯）
 * @param   r     红色亮度 0-255
 * @param   g     绿色亮度 0-255
 * @param   b     蓝色亮度 0-255
 * @note    需调用 WS2812_Send() 刷新到灯带
 */
void WS2812_SetLED(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_NUM_LEDS) return;
    led_color_buf[index].r = r;
    led_color_buf[index].g = g;
    led_color_buf[index].b = b;
}

/**
 * @brief   设置所有 12 颗 LED 为相同颜色（不立即发送）
 * @note    需调用 WS2812_Send() 刷新到灯带
 */
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < WS2812_NUM_LEDS; i++)
    {
        led_color_buf[i].r = r;
        led_color_buf[i].g = g;
        led_color_buf[i].b = b;
    }
}

/**
 * @brief   通过 DMA+PWM 将颜色缓冲发送到 12 灯灯带（非阻塞）
 * @note    DMA 硬件自动完成传输（~423µs），CPU 立即返回。
 *          调用间隔 50ms >> 423µs，下次调用时 DMA 早已完成。
 *          - 发送前自动编码 LED 颜色到 CCR 缓冲区
 *          - 第一个 CCR 值预装入 CCR1 寄存器，DMA 从第二个值开始传输
 *          - DMA 传完后 TIM2 继续以 CCR=0 运行 → 输出低电平 → 灯带自动锁存
 */
void WS2812_Send(void)
{
    if (!pwm_ready) return;

    /*--- 0. 清理上次传输（上次 DMA 早在 423µs 内完成，50ms 后才来下一次）---*/
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
    WS2812_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    while (WS2812_DMA_STREAM->CR & DMA_SxCR_EN);

    /*--- 1. 将颜色数据编码为 CCR 值填充 DMA 缓冲区 ---*/
    WS2812_EncodeBuffer();

    /*--- 2. 软件预装第一个 CCR 值 → 第一周期就用正确的占空比 ---*/
    WS2812_TIM_INSTANCE->CCR1 = ccr_dma_buf[0];

    /*--- 3. 配置 DMA 传输 ---*/
    /* 内存地址从 ccr_dma_buf[1] 开始（[0] 已预装入 CCR1） */
    WS2812_DMA_STREAM->M0AR = (uint32_t)&ccr_dma_buf[1];
    /* 剩余传输次数 = TOTAL_BITS - 1 */
    WS2812_DMA_STREAM->NDTR = WS2812_TOTAL_BITS - 1;

    /* 清除 DMA1_Stream0 所有中断标志（Stream 0-3 → LIFCR，Stream 4-7 → HIFCR） */
    DMA1->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0
                | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

    /*--- 4. 使能 TIM2_CH1 DMA 请求 + 启动 DMA Stream，立即返回 ---*/
    __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
    WS2812_DMA_STREAM->CR |= DMA_SxCR_EN;

    /* 不等待！DMA 硬件在后台自动搬运 338 个 CCR 值，
       420µs 后灯带自动锁存，下次调用再做清理。 */
}

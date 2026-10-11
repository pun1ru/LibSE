#include "WS2812.h"

#include <stddef.h>

static SPI_HandleTypeDef *ws2812_spi;

typedef struct {
    uint8_t active;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t last_level;
    uint32_t period_ms;
    uint32_t start_tick;
} WS2812_BreatheState_t;

static WS2812_BreatheState_t breathe_state;

#define WS2812_SPI_LOW  0x60U
#define WS2812_SPI_HIGH 0x78U

void WS2812_SPI_Init(SPI_HandleTypeDef *hspi)
{
    ws2812_spi = hspi;
}

void WS2812_SPI_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];

    if (ws2812_spi == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < 8U; ++i) {
        txbuf[7U - i] = ((g >> i) & 1U) != 0U ? WS2812_SPI_HIGH : WS2812_SPI_LOW;
        txbuf[15U - i] = ((r >> i) & 1U) != 0U ? WS2812_SPI_HIGH : WS2812_SPI_LOW;
        txbuf[23U - i] = ((b >> i) & 1U) != 0U ? WS2812_SPI_HIGH : WS2812_SPI_LOW;
    }

    (void)HAL_SPI_Transmit(ws2812_spi, txbuf, sizeof(txbuf), 0xFFFFU);
}

void WS2812_Breathe_Start(uint8_t r, uint8_t g, uint8_t b,
                          uint32_t period_ms)
{
    breathe_state.active = 1U;
    breathe_state.r = r;
    breathe_state.g = g;
    breathe_state.b = b;
    breathe_state.last_level = 0xFFU;
    breathe_state.period_ms = (period_ms < 2U) ? 2U : period_ms;
    breathe_state.start_tick = HAL_GetTick();
    WS2812_Breathe_Update();
}

void WS2812_Breathe_Stop(void)
{
    breathe_state.active = 0U;
    breathe_state.last_level = 0U;
    WS2812_SPI_Ctrl(0U, 0U, 0U);
}

void WS2812_Breathe_Update(void)
{
    uint32_t elapsed;
    uint32_t phase;
    uint32_t half_period;
    uint8_t level;

    if (breathe_state.active == 0U) {
        return;
    }

    elapsed = HAL_GetTick() - breathe_state.start_tick;
    phase = elapsed % breathe_state.period_ms;
    half_period = breathe_state.period_ms / 2U;

    if (phase < half_period) {
        level = (uint8_t)((phase * 255U) / half_period);
    } else {
        level = (uint8_t)(((breathe_state.period_ms - phase) * 255U) /
                          (breathe_state.period_ms - half_period));
    }

    if (level == breathe_state.last_level) {
        return;
    }

    breathe_state.last_level = level;
    WS2812_SPI_Ctrl(
        (uint8_t)(((uint16_t)breathe_state.r * level + 127U) / 255U),
        (uint8_t)(((uint16_t)breathe_state.g * level + 127U) / 255U),
        (uint8_t)(((uint16_t)breathe_state.b * level + 127U) / 255U));
}

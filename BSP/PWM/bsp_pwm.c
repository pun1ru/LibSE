#include "bsp_pwm.h"

void BSP_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)HAL_TIM_PWM_Start(htim, channel);
}

void BSP_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)HAL_TIM_PWM_Stop(htim, channel);
}

void BSP_PWM_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse)
{
    uint64_t period;
    period = (uint64_t)__HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    if ((uint64_t)pulse > period ||
        (!IS_TIM_32B_COUNTER_INSTANCE(htim->Instance) && pulse > UINT16_MAX)) {
        return;
    }
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

void BSP_PWM_SetDutyPermille(TIM_HandleTypeDef *htim, uint32_t channel,
                             uint16_t duty_permille)
{
    uint64_t pulse;
    pulse = (((uint64_t)__HAL_TIM_GET_AUTORELOAD(htim) + 1U) * duty_permille + 500U) / 1000U;
    if (pulse > UINT32_MAX) {
        return;
    }
    BSP_PWM_SetPulse(htim, channel, (uint32_t)pulse);
}

void BSP_PWM_SetFrequency(TIM_HandleTypeDef *htim, uint32_t timer_clock_hz,
                          uint32_t frequency_hz)
{
    if (frequency_hz == 0U) {
        return;
    }

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    uint32_t divider = (uint32_t)(((uint64_t)timer_clock_hz +
                                   (uint64_t)period * frequency_hz / 2U) /
                                  ((uint64_t)period * frequency_hz));
    if (divider == 0U || divider > 65536U) {
        return;
    }
    __HAL_TIM_SET_PRESCALER(htim, divider - 1U);
    __HAL_TIM_SET_COUNTER(htim, 0U);
    htim->Instance->EGR = TIM_EGR_UG;
}

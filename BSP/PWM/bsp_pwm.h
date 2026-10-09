#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "stm32h7xx_hal.h"

/* Call MX_TIMx_Init first. Duty is in 0..1000 permille. */
void BSP_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);
void BSP_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel);
void BSP_PWM_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse);
void BSP_PWM_SetDutyPermille(TIM_HandleTypeDef *htim, uint32_t channel,
                             uint16_t duty_permille);

#endif

#include "stir.h"

#include <math.h>
#include <string.h>

#define STIR_FULL_TURN_DEG   360.0f

/*--------------------------------------堵转检测--------------------------------------*/
/* 低速高扭矩累计计数，其余衰减；到上限置堵转，衰减到 0 才清除，避免在判据边缘抖动 */
static void Stir_DetectBlock(Stir_t *stir)
{
    StirObserver_t *obs = &stir->observer;
    const StirBlockParam_t *param = &stir->config.block;

    if (fabsf(obs->omega_dps) < param->speed_max_dps &&
        fabsf(obs->torque_nm) > param->torque_min_nm)
    {
        uint32_t count = (uint32_t)obs->block_count + param->count_up;
        obs->block_count = count > param->count_limit ? param->count_limit : (uint16_t)count;
    }
    else if (obs->block_count > param->count_down)
    {
        obs->block_count -= param->count_down;
    }
    else
    {
        obs->block_count = 0U;
    }

    if (obs->block_count >= param->count_limit)
    {
        obs->blocked = 1U;
    }
    else if (obs->block_count == 0U)
    {
        obs->blocked = 0U;
    }
}

/*--------------------------------------初始化--------------------------------------*/
void Stir_Init(Stir_t *stir, const StirConfig_t *config)
{
    memset(stir, 0, sizeof(*stir));
    stir->config = *config;
    stir->state = config->init_state;

    StirController_t *ctrl = &stir->controller;
    const StirGain_t *gain = &config->gain;
    PIDInitialize(&ctrl->angle_pid, gain->angle.kp, gain->angle.ki, gain->angle.kd,
                  gain->angle.d_alpha, gain->angle.sum_error_max, gain->angle.output_max);
    PIDInitialize(&ctrl->speed_pid, gain->speed.kp, gain->speed.ki, gain->speed.kd,
                  gain->speed.d_alpha, gain->speed.sum_error_max, gain->speed.output_max);
    RateDividerInit(&ctrl->angle_loop, config->angle_loop_div);
}

/*--------------------------------------输入层--------------------------------------*/
void Stir_Input(Stir_t *stir, const StirInput_t *input)
{
    /* 无论什么状态都更新请求锁存，保护中或刚退出保护时按住的请求不会在之后触发 */
    uint8_t fire_edge = (input->fire_request != 0U && stir->fire_hold == 0U);
    stir->fire_hold = (input->fire_request != 0U);

    if (input->state == STIR_PROTECTED)
    {
        stir->state = STIR_PROTECTED;
        return;
    }
    if (stir->state == STIR_PROTECTED)
    {
        /* 退出保护，目标已由控制层对齐观测，具体状态等控制层判定 */
        stir->state = STIR_MOVING;
        return;
    }

    /* 只在到位时接受上升沿请求，目标最多比反馈超前一个弹位 */
    if (fire_edge && stir->state == STIR_LOCATED)
    {
        stir->target_deg -= STIR_FULL_TURN_DEG / (float)stir->config.pocket_count;
        stir->shot_count++;
        stir->state = STIR_MOVING;
    }
}

/*--------------------------------------估计层--------------------------------------*/
void Stir_Estimate(Stir_t *stir)
{
    /* TODO: 反馈折算方向、跨界解包成连续角度 */
    Stir_DetectBlock(stir);
}

/*--------------------------------------控制层--------------------------------------*/
void Stir_Control(Stir_t *stir)
{
    /* TODO: MOVING / LOCATED / BLOCKED 判定、双环 PID、写电机指令 */
    (void)stir;
}

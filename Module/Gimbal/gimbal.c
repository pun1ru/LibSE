#include "gimbal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define GIMBAL_COS_PITCH_MIN    0.05f   /* 防止 pitch 接近 ±90° 时 1/cos 发散 */

static void Gimbal_LoadGain(Gimbal_t *gimbal, const GimbalGainSet_t *gain)
{
    for (uint8_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
    {
        const GimbalAxisParam_t *param = &gain->axis[i];
        GimbalAxisCtrl_t *ctrl = &gimbal->controller.axis[i];
        PIDInitialize(&ctrl->angle_pid, param->angle.kp, param->angle.ki,
                      param->angle.kd, param->angle.sum_error_max,
                      param->angle.output_max);
        PIDInitialize(&ctrl->speed_pid, param->speed.kp, param->speed.ki,
                      param->speed.kd, param->speed.sum_error_max,
                      param->speed.output_max);
        PIDReset(&ctrl->angle_pid);
        PIDReset(&ctrl->speed_pid);
        ctrl->speed_ref_rads = 0.0f;
        ctrl->feedforward = 0.0f;
        ctrl->output = 0.0f;
    }
}

/* 按电机当前控制模式把输出写到对应字段 */
static void Gimbal_ApplyOutput(Motor_t *motor, float output)
{
    if (motor == NULL)
    {
        return;
    }

    MotorCmd_t *cmd = &motor->motorcmd;
    switch (cmd->control_mode)
    {
    case MODE_MIT:
        cmd->position = 0.0f;
        cmd->velocity = 0.0f;
        cmd->kp = 0.0f;
        cmd->kd = 0.0f;
        cmd->torque = output;
        break;
    case MODE_TORQUE:
        cmd->torque = output;
        break;
    case MODE_CURRENT:
        cmd->current = output;
        break;
    default:
        break;
    }
}

static void Gimbal_ZeroOutput(Gimbal_t *gimbal)
{
    for (uint8_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
    {
        GimbalAxisCtrl_t *ctrl = &gimbal->controller.axis[i];
        PIDReset(&ctrl->angle_pid);
        PIDReset(&ctrl->speed_pid);
        ctrl->speed_ref_rads = 0.0f;
        ctrl->feedforward = 0.0f;
        ctrl->output = 0.0f;
        Gimbal_ApplyOutput(gimbal->motor[i], 0.0f);
    }
}

void Gimbal_Init(Gimbal_t *gimbal, const GimbalConfig_t *config)
{
    if (gimbal == NULL || config == NULL)
    {
        return;
    }

    memset(gimbal, 0, sizeof(*gimbal));
    gimbal->model = config->model;
    gimbal->motor[GIMBAL_YAW] = config->yaw_motor;
    gimbal->motor[GIMBAL_PITCH] = config->pitch_motor;
    gimbal->normal_gain = config->normal_gain;
    gimbal->sniper_gain = config->sniper_gain;
    gimbal->ins = config->ins;
    gimbal->pitch_min_rad = config->pitch_min_rad;
    gimbal->pitch_max_rad = config->pitch_max_rad;
    gimbal->state = config->init_state;
    SmoothFilterInitialize(&gimbal->observer.pitch_alpha_filter, config->alpha_filter);
    SmoothFilterInitialize(&gimbal->observer.yaw_alpha_filter, config->alpha_filter);
    Gimbal_LoadGain(gimbal, gimbal->state == GIMBAL_SNIPER ?
                            &gimbal->sniper_gain : &gimbal->normal_gain);
}

/*--------------------------------------输入层--------------------------------------*/
void Gimbal_Input(Gimbal_t *gimbal, const GimbalInput_t *input)
{
    if (gimbal == NULL || input == NULL)
    {
        return;
    }

    if (input->state != gimbal->state)
    {
        gimbal->state = input->state;
        if (input->state == GIMBAL_PROTECTED)
        {
            Gimbal_ZeroOutput(gimbal);
        }
        else
        {
            Gimbal_LoadGain(gimbal, input->state == GIMBAL_SNIPER ?
                                    &gimbal->sniper_gain : &gimbal->normal_gain);
        }
    }

    /* 保护状态下目标由控制层跟随观测，忽略外部角度 */
    if (gimbal->state == GIMBAL_PROTECTED)
    {
        return;
    }
    gimbal->target.pitch_rad = DoubleEdgeLimiter(input->pitch_rad,
                                                 gimbal->pitch_min_rad,
                                                 gimbal->pitch_max_rad);
    gimbal->target.yaw_rad = AngleLimit(input->yaw_rad, -MOTOR_PI, MOTOR_PI);
    gimbal->target.pitch_omega_rads = input->pitch_omega_rads;
    gimbal->target.yaw_omega_rads = input->yaw_omega_rads;
}

/*--------------------------------------估计层--------------------------------------*/
void Gimbal_Estimate(Gimbal_t *gimbal, float dt_s)
{
    if (gimbal == NULL)
    {
        return;
    }

    GimbalObserver_t *obs = &gimbal->observer;
    INS_Euler_t euler;
    float gyro[3];

    if (INS_GetEuler(gimbal->ins, &euler) == 0U ||
        INS_GetGyro(gimbal->ins, gyro) == 0U)
    {
        obs->valid = 0U;
        return;
    }

    obs->roll_rad = euler.roll_deg * MOTOR_DEG_TO_RAD;
    obs->pitch_rad = euler.pitch_deg * MOTOR_DEG_TO_RAD;
    obs->yaw_rad = euler.yaw_deg * MOTOR_DEG_TO_RAD;

    /* ZYX 欧拉角速率：机体角速度 -> d(pitch)/dt, d(yaw)/dt */
    float sin_roll = sinf(obs->roll_rad);
    float cos_roll = cosf(obs->roll_rad);
    float cos_pitch = cosf(obs->pitch_rad);
    if (fabsf(cos_pitch) < GIMBAL_COS_PITCH_MIN)
    {
        cos_pitch = cos_pitch < 0.0f ? -GIMBAL_COS_PITCH_MIN : GIMBAL_COS_PITCH_MIN;
    }
    float pitch_omega = cos_roll * gyro[1] - sin_roll * gyro[2];
    float yaw_omega = (sin_roll * gyro[1] + cos_roll * gyro[2]) / cos_pitch;

    if (obs->valid != 0U && dt_s > 0.0f)
    {
        obs->pitch_alpha_rads2 = SmoothFilterUpdate(&obs->pitch_alpha_filter,
                                                    (pitch_omega - obs->pitch_omega_rads) / dt_s);
        obs->yaw_alpha_rads2 = SmoothFilterUpdate(&obs->yaw_alpha_filter,
                                                  (yaw_omega - obs->yaw_omega_rads) / dt_s);
    }
    else
    {
        SmoothFilterInitialize(&obs->pitch_alpha_filter, obs->pitch_alpha_filter.alpha);
        SmoothFilterInitialize(&obs->yaw_alpha_filter, obs->yaw_alpha_filter.alpha);
        obs->pitch_alpha_rads2 = 0.0f;
        obs->yaw_alpha_rads2 = 0.0f;
    }
    obs->pitch_omega_rads = pitch_omega;
    obs->yaw_omega_rads = yaw_omega;

    if (gimbal->motor[GIMBAL_YAW] != NULL)
    {
        obs->yaw_motor_angle_rad = gimbal->motor[GIMBAL_YAW]->feedback.abs_angle_rad;
    }
    if (gimbal->motor[GIMBAL_PITCH] != NULL)
    {
        obs->pitch_motor_angle_rad = gimbal->motor[GIMBAL_PITCH]->feedback.abs_angle_rad;
    }
    obs->valid = 1U;
}

/*--------------------------------------控制层--------------------------------------*/
/* 角度环 -> 速度参考（加目标角速度前馈）-> 速度环 + 模型前馈 */
static float Gimbal_AxisControl(GimbalAxisCtrl_t *ctrl, float angle_error_rad,
                                float target_omega_rads, float omega_rads)
{
    ctrl->speed_ref_rads = PIDUpdate(&ctrl->angle_pid, angle_error_rad) +
                           target_omega_rads;
    ctrl->feedforward = 0.0f;   /* TODO: 由 model 计算重力/惯量前馈 */
    ctrl->output = AbsLimiter(PIDUpdate(&ctrl->speed_pid,
                                        ctrl->speed_ref_rads - omega_rads) +
                              ctrl->feedforward,
                              ctrl->speed_pid.output_max);
    return ctrl->output;
}

void Gimbal_Control(Gimbal_t *gimbal)
{
    if (gimbal == NULL)
    {
        return;
    }

    const GimbalObserver_t *obs = &gimbal->observer;
    if (gimbal->state == GIMBAL_PROTECTED || obs->valid == 0U)
    {
        Gimbal_ZeroOutput(gimbal);
        /* 目标跟随当前姿态，上层以 gimbal->target 为增量基准，恢复时不跳变 */
        if (obs->valid != 0U)
        {
            gimbal->target.pitch_rad = DoubleEdgeLimiter(obs->pitch_rad,
                                                         gimbal->pitch_min_rad,
                                                         gimbal->pitch_max_rad);
            gimbal->target.yaw_rad = obs->yaw_rad;
        }
        gimbal->target.pitch_omega_rads = 0.0f;
        gimbal->target.yaw_omega_rads = 0.0f;
        return;
    }

    const GimbalTarget_t *target = &gimbal->target;
    float yaw_out = Gimbal_AxisControl(&gimbal->controller.axis[GIMBAL_YAW],
                                       AngleLimit(target->yaw_rad - obs->yaw_rad, -MOTOR_PI, MOTOR_PI),
                                       target->yaw_omega_rads, obs->yaw_omega_rads);
    float pitch_out = Gimbal_AxisControl(&gimbal->controller.axis[GIMBAL_PITCH],
                                         target->pitch_rad - obs->pitch_rad,
                                         target->pitch_omega_rads, obs->pitch_omega_rads);
    Gimbal_ApplyOutput(gimbal->motor[GIMBAL_YAW], yaw_out);
    Gimbal_ApplyOutput(gimbal->motor[GIMBAL_PITCH], pitch_out);
}

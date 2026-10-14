#include "gimbal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define GIMBAL_COS_PITCH_MIN    0.05f   /* 防止 pitch 接近 ±90° 时 1/cos 发散 */

/* 清空控制器状态并让角度环下个周期立即执行；输出由 Gimbal_Control 末尾统一下发 */
static void Gimbal_ResetControl(Gimbal_t *gimbal)
{
    for (uint8_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
    {
        GimbalAxisCtrl_t *ctrl = &gimbal->controller.axis[i];
        PIDReset(&ctrl->angle_pid);
        PIDReset(&ctrl->speed_pid);
        ctrl->angle_out_dps = 0.0f;
        ctrl->speed_ref_dps = 0.0f;
        ctrl->feedforward = 0.0f;
        ctrl->output = 0.0f;
    }
    RateDividerReset(&gimbal->controller.angle_loop);
}

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
    }
    Gimbal_ResetControl(gimbal);
}

/* 按电机当前控制模式把输出写到对应字段 */
static void Gimbal_WriteCommand(Motor_t *motor, float output)
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
/* 先写完所有轴再统一发送：同组 RM 电机共用一帧，逐个写+发会带出旧值 */
static void Gimbal_ApplyOutput(Gimbal_t *gimbal)
{
    for (uint8_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
    {
        Gimbal_WriteCommand(gimbal->motor[i], gimbal->controller.axis[i].output);
    }
    for (uint8_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
    {
        Motor_t *motor = gimbal->motor[i];
        if (motor != NULL)
        {
            motor->motorops.sendcommand(motor);
        }
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
    gimbal->pitch_min_deg = config->pitch_min_deg;
    gimbal->pitch_max_deg = config->pitch_max_deg;
    gimbal->state = config->init_state;
    SmoothFilterInitialize(&gimbal->observer.pitch_alpha_filter, config->alpha_filter);
    SmoothFilterInitialize(&gimbal->observer.yaw_alpha_filter, config->alpha_filter);
    RateDividerInit(&gimbal->controller.angle_loop, config->angle_loop_div);
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
            Gimbal_ResetControl(gimbal);
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
    gimbal->target.pitch_deg = DoubleEdgeLimiter(input->pitch_deg,
                                                 gimbal->pitch_min_deg,
                                                 gimbal->pitch_max_deg);
    gimbal->target.yaw_deg = AngleLimit(input->yaw_deg, -180.0f, 180.0f);
    gimbal->target.pitch_omega_dps = input->pitch_omega_dps;
    gimbal->target.yaw_omega_dps = input->yaw_omega_dps;
}

/*--------------------------------------估计层--------------------------------------*/
void Gimbal_Estimate(Gimbal_t *gimbal, float dt_s)
{
    GimbalObserver_t *obs = &gimbal->observer;
    INS_Euler_t euler;
    float gyro[3];

    if (INS_GetEuler(gimbal->ins, &euler) == 0U ||
        INS_GetGyro(gimbal->ins, gyro) == 0U)
    {
        obs->valid = 0U;
        return;
    }

    obs->roll_deg = euler.roll_deg;
    obs->pitch_deg = euler.pitch_deg;
    obs->yaw_deg = euler.yaw_deg;

    /* ZYX 欧拉角速率：机体角速度 -> d(pitch)/dt, d(yaw)/dt
     * 三角函数用 rad，陀螺仪为 rad/s，结果转 deg/s */
    float roll_rad = obs->roll_deg * MOTOR_DEG_TO_RAD;
    float sin_roll = sinf(roll_rad);
    float cos_roll = cosf(roll_rad);
    float cos_pitch = cosf(obs->pitch_deg * MOTOR_DEG_TO_RAD);
    if (fabsf(cos_pitch) < GIMBAL_COS_PITCH_MIN)
    {
        cos_pitch = cos_pitch < 0.0f ? -GIMBAL_COS_PITCH_MIN : GIMBAL_COS_PITCH_MIN;
    }
    float pitch_omega = (cos_roll * gyro[1] - sin_roll * gyro[2]) * MOTOR_RAD_TO_DEG;
    float yaw_omega = (sin_roll * gyro[1] + cos_roll * gyro[2]) / cos_pitch *
                      MOTOR_RAD_TO_DEG;

    if (obs->valid != 0U && dt_s > 0.0f)
    {
        obs->pitch_alpha_dps2 = SmoothFilterUpdate(&obs->pitch_alpha_filter,
                                                    (pitch_omega - obs->pitch_omega_dps) / dt_s);
        obs->yaw_alpha_dps2 = SmoothFilterUpdate(&obs->yaw_alpha_filter,
                                                  (yaw_omega - obs->yaw_omega_dps) / dt_s);
    }
    else
    {
        SmoothFilterInitialize(&obs->pitch_alpha_filter, obs->pitch_alpha_filter.alpha);
        SmoothFilterInitialize(&obs->yaw_alpha_filter, obs->yaw_alpha_filter.alpha);
        obs->pitch_alpha_dps2 = 0.0f;
        obs->yaw_alpha_dps2 = 0.0f;
    }
    obs->pitch_omega_dps = pitch_omega;
    obs->yaw_omega_dps = yaw_omega;
    obs->yaw_motor_angle_deg = gimbal->motor[GIMBAL_YAW]->feedback.abs_angle_deg;
    obs->pitch_motor_angle_deg = gimbal->motor[GIMBAL_PITCH]->feedback.abs_angle_deg;
    obs->valid = 1U;
}

/*--------------------------------------控制层--------------------------------------*/
/* 外环（分频）：角度误差 deg -> 角速度 deg/s，结果保持到下次执行 */
static void Gimbal_AngleLoop(GimbalAxisCtrl_t *ctrl, float angle_error_deg)
{
    ctrl->angle_out_dps = PIDUpdate(&ctrl->angle_pid, angle_error_deg);
}

/* 内环（每周期）：角度环输出 + 目标角速度前馈 -> 速度环 + 模型前馈 */
static void Gimbal_SpeedLoop(GimbalAxisCtrl_t *ctrl, float target_omega_dps,
                             float omega_dps)
{
    ctrl->speed_ref_dps = ctrl->angle_out_dps + target_omega_dps;
    ctrl->feedforward = 0.0f;   /* TODO: 由 model 计算重力/惯量前馈 */
    ctrl->output = AbsLimiter(PIDUpdate(&ctrl->speed_pid,
                                        ctrl->speed_ref_dps - omega_dps) +
                              ctrl->feedforward,
                              ctrl->speed_pid.output_max);
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
        Gimbal_ResetControl(gimbal);
        /* 目标跟随当前姿态，上层以 gimbal->target 为增量基准，恢复时不跳变 */
        if (obs->valid != 0U)
        {
            gimbal->target.pitch_deg = DoubleEdgeLimiter(obs->pitch_deg,
                                                         gimbal->pitch_min_deg,
                                                         gimbal->pitch_max_deg);
            gimbal->target.yaw_deg = obs->yaw_deg;
        }
        gimbal->target.pitch_omega_dps = 0.0f;
        gimbal->target.yaw_omega_dps = 0.0f;
        Gimbal_ApplyOutput(gimbal);
        return;
    }

    const GimbalTarget_t *target = &gimbal->target;
    GimbalAxisCtrl_t *yaw = &gimbal->controller.axis[GIMBAL_YAW];
    GimbalAxisCtrl_t *pitch = &gimbal->controller.axis[GIMBAL_PITCH];
    if (RateDividerTick(&gimbal->controller.angle_loop) != 0U)
    {
        Gimbal_AngleLoop(yaw, AngleLimit(target->yaw_deg - obs->yaw_deg,
                                         -180.0f, 180.0f));
        Gimbal_AngleLoop(pitch, target->pitch_deg - obs->pitch_deg);
    }
    Gimbal_SpeedLoop(yaw, target->yaw_omega_dps, obs->yaw_omega_dps);
    Gimbal_SpeedLoop(pitch, target->pitch_omega_dps, obs->pitch_omega_dps);
    Gimbal_ApplyOutput(gimbal);
}

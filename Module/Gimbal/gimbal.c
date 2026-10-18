#include "gimbal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "position_tran.h"

#define GIMBAL_HORIZON_MIN      0.02f   /* 指向向量水平投影下限，目标接近天顶时冻结 yaw */
#define GIMBAL_WORLD_PITCH_MAX  90.0f   /* 世界系 pitch 目标的物理范围 */
#define GIMBAL_YAW_WRAP_DEG     180.0f  /* yaw 角度归一化到 [-180, 180] */

/*--------------------------------------指向向量转换--------------------------------------*/
/* 世界系目标（INS 约定）-> 世界系单位指向向量及其变化率
 * 变化率直接乘 deg/s，后续逆解出的关节角速度即为 deg/s */
static void Gimbal_TargetToWorld(const GimbalTarget_t *target, float dir[3], float rate[3])
{
    float cy = cosf(target->yaw_deg * MOTOR_DEG_TO_RAD);
    float sy = sinf(target->yaw_deg * MOTOR_DEG_TO_RAD);
    float cp = cosf(target->pitch_deg * MOTOR_DEG_TO_RAD);
    float sp = sinf(target->pitch_deg * MOTOR_DEG_TO_RAD);
    //只关心x方向投影向量作为目标
    dir[0] = cy * cp;
    dir[1] = -sy * cp;
    dir[2] = sp;
    rate[0] = -sy * cp * target->yaw_omega_dps - cy * sp * target->pitch_omega_dps;
    rate[1] = -cy * cp * target->yaw_omega_dps + sy * sp * target->pitch_omega_dps;
    rate[2] = cp * target->pitch_omega_dps;
}

/* 世界系单位指向向量 -> 世界系目标（INS 约定），用于限位后回写可达方向 */
static void Gimbal_WorldToTarget(const float dir[3], GimbalTarget_t *target)
{
    target->yaw_deg = -atan2f(dir[1], dir[0]) * MOTOR_RAD_TO_DEG;
    target->pitch_deg = atan2f(dir[2], sqrtf(dir[0] * dir[0] + dir[1] * dir[1])) *
                        MOTOR_RAD_TO_DEG;
}
/*--------------------------------------控制器管理--------------------------------------*/
/* 清空控制器状态并让角度环下个周期立即执行；关节指令对齐当前关节角，恢复时不跳变 */
static void Gimbal_ResetControl(Gimbal_t *gimbal)
{
    GimbalJointRef_t *ref = &gimbal->controller.joint_ref;
    for (uint8_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
    {
        GimbalAxisCtrl_t *ctrl = &gimbal->controller.axis[i];
        PIDReset(&ctrl->angle_pid);
        PIDReset(&ctrl->speed_pid);
        ctrl->angle_out_dps = 0.0f;
        ctrl->speed_ref_dps = 0.0f;
        ctrl->feedforward = 0.0f;
        ctrl->output = 0.0f;
        ref->angle_deg[i] = gimbal->observer.joint_deg[i];
        ref->omega_dps[i] = 0.0f;
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
                      param->angle.kd, param->angle.d_alpha,
                      param->angle.sum_error_max, param->angle.output_max);
        PIDInitialize(&ctrl->speed_pid, param->speed.kp, param->speed.ki,
                      param->speed.kd, param->speed.d_alpha,
                      param->speed.sum_error_max, param->speed.output_max);
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
    RateDividerInit(&gimbal->controller.angle_loop, config->angle_loop_div);
    Gimbal_LoadGain(gimbal, gimbal->state == GIMBAL_SNIPER ?
                            &gimbal->sniper_gain : &gimbal->normal_gain);
}

/*--------------------------------------输入层--------------------------------------*/
void Gimbal_Input(Gimbal_t *gimbal, const GimbalInput_t *input)
{
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
    /* 关节限位依赖底盘姿态，留给控制层在关节空间截断 */
    gimbal->target.pitch_deg = DoubleEdgeLimiter(input->pitch_deg,
                                                 -GIMBAL_WORLD_PITCH_MAX,
                                                 GIMBAL_WORLD_PITCH_MAX);
    gimbal->target.yaw_deg = AngleLimit(input->yaw_deg, -GIMBAL_YAW_WRAP_DEG,
                                        GIMBAL_YAW_WRAP_DEG);
    gimbal->target.pitch_omega_dps = input->pitch_omega_dps;
    gimbal->target.yaw_omega_dps = input->yaw_omega_dps;
}

/*--------------------------------------估计层--------------------------------------*/
/* 电机 inverse 只翻转指令，反馈在这里按同一方向折算成关节角 */
static float Gimbal_MotorDirection(const Motor_t *motor)
{
    return motor->inverse ? -1.0f : 1.0f;
}

void Gimbal_Estimate(Gimbal_t *gimbal)
{
    GimbalObserver_t *obs = &gimbal->observer;
    float quat[4];

    if (INS_GetQuat(gimbal->ins, quat) == 0U)
    {
        obs->valid = 0U;
        return;
    }

    for (uint8_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
    {
        const Motor_t *motor = gimbal->motor[i];
        float direction = Gimbal_MotorDirection(motor);
        obs->joint_deg[i] = direction * motor->feedback.abs_angle_deg;
        obs->joint_omega_dps[i] = direction * motor->feedback.omega_rads * MOTOR_RAD_TO_DEG;
    }

    const float quat_flu[4] = {quat[0], quat[1], -quat[2], -quat[3]};
    Quat_ToMatrix(quat_flu, obs->gimbal_to_world);

    /* 关节角 -> 末端到底盘的旋转矩阵：DH 正运动学 T03 = T01 * T12 * T23 取旋转部分
     * 按 gimbal.md 的参数表结果为 Rz(yaw) * Ry(-pitch)；末端行不对应关节，q 恒为 0 */
    const float q_deg[GIMBAL_FRAME_COUNT] = {obs->joint_deg[GIMBAL_YAW],
                                             obs->joint_deg[GIMBAL_PITCH], 0.0f};
    float T[4][4];
    DH_ForwardKinematics(gimbal->model.dh, q_deg, GIMBAL_FRAME_COUNT, T);
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        memcpy(obs->gimbal_to_base[i], T[i], sizeof(obs->gimbal_to_base[i]));
    }

    /* 末端世界指向（INS 约定）取枪管 x 轴在世界系的方向，天顶处 yaw 退化为 0，pitch 仍正确 */
    const float barrel_world[3] = {obs->gimbal_to_world[0][0], obs->gimbal_to_world[1][0],
                                   obs->gimbal_to_world[2][0]};
    GimbalTarget_t pointing;
    Gimbal_WorldToTarget(barrel_world, &pointing);
    obs->pitch_deg = pointing.pitch_deg;
    obs->yaw_deg = pointing.yaw_deg;

    /* 末端 IMU + 编码器反解底盘姿态：base_to_world = gimbal_to_world * gimbal_to_base^T */
    Mat3_MulT(obs->gimbal_to_world, obs->gimbal_to_base, obs->base_to_world);
    obs->valid = 1U;
}

/*--------------------------------------解算层--------------------------------------*/
/* 世界系目标 -> 底盘系指向 -> 关节角指令（只对齐末端 x 轴，目标 roll 不参与）
 * pitch 超出关节限位时截断，并把目标改写为限位处的世界指向，避免增量控制越积越多 */
static void Gimbal_SolveJoint(Gimbal_t *gimbal)
{
    const GimbalObserver_t *obs = &gimbal->observer;
    GimbalJointRef_t *ref = &gimbal->controller.joint_ref;
    float dir_world[3];
    float rate_world[3];
    float dir_base[3];
    float rate_base[3];

    Gimbal_TargetToWorld(&gimbal->target, dir_world, rate_world);
    Mat3_TransMulVec(obs->base_to_world, dir_world, dir_base);
    Mat3_TransMulVec(obs->base_to_world, rate_world, rate_base);

    float horizon_sq = dir_base[0] * dir_base[0] + dir_base[1] * dir_base[1];
    float horizon = sqrtf(horizon_sq);
    if (horizon > GIMBAL_HORIZON_MIN)
    {
        ref->angle_deg[GIMBAL_YAW] = atan2f(dir_base[1], dir_base[0]) * MOTOR_RAD_TO_DEG;
        ref->omega_dps[GIMBAL_YAW] = (dir_base[0] * rate_base[1] - dir_base[1] * rate_base[0]) /
                                     horizon_sq;
        ref->omega_dps[GIMBAL_PITCH] = rate_base[2] / horizon;
    }
    else
    {
        /* 目标接近天顶，yaw 不确定，保持上一周期指令 */
        ref->omega_dps[GIMBAL_YAW] = 0.0f;
        ref->omega_dps[GIMBAL_PITCH] = 0.0f;
    }

    float pitch_cmd = atan2f(dir_base[2], horizon) * MOTOR_RAD_TO_DEG;
    ref->angle_deg[GIMBAL_PITCH] = DoubleEdgeLimiter(pitch_cmd, gimbal->pitch_min_deg,
                                                     gimbal->pitch_max_deg);
    if (ref->angle_deg[GIMBAL_PITCH] != pitch_cmd)
    {
        /* 限位处的枪管 x 轴 = DH 正运动学 T 的第一列，再转到世界系 */
        const float q_deg[GIMBAL_FRAME_COUNT] = {ref->angle_deg[GIMBAL_YAW],
                                                 ref->angle_deg[GIMBAL_PITCH], 0.0f};
        float T[4][4];
        DH_ForwardKinematics(gimbal->model.dh, q_deg, GIMBAL_FRAME_COUNT, T);
        const float reach_base[3] = {T[0][0], T[1][0], T[2][0]};
        float reach_world[3];
        Mat3_MulVec(obs->base_to_world, reach_base, reach_world);
        Gimbal_WorldToTarget(reach_world, &gimbal->target);
        ref->omega_dps[GIMBAL_PITCH] = 0.0f;
    }
}

/*--------------------------------------控制层--------------------------------------*/
/* 外环（分频）：关节角误差 deg -> 关节角速度 deg/s，结果保持到下次执行 */
static void Gimbal_AngleLoop(GimbalAxisCtrl_t *ctrl, float angle_error_deg)
{
    ctrl->angle_out_dps = PIDUpdate(&ctrl->angle_pid, angle_error_deg);
}

/* 内环（每周期）：角度环输出 + 关节角速度前馈 -> 速度环 + 模型前馈 */
static void Gimbal_SpeedLoop(GimbalAxisCtrl_t *ctrl, float ref_omega_dps,
                             float omega_dps)
{
    ctrl->speed_ref_dps = ctrl->angle_out_dps + ref_omega_dps;
    ctrl->feedforward = 0.0f;   /* TODO: 由 model 计算重力/惯量前馈 */
    ctrl->output = AbsLimiter(PIDUpdate(&ctrl->speed_pid,
                                                     ctrl->speed_ref_dps - omega_dps) +
                              ctrl->feedforward,
                              ctrl->speed_pid.output_max);
}

void Gimbal_Control(Gimbal_t *gimbal)
{
    const GimbalObserver_t *obs = &gimbal->observer;
    if (gimbal->state == GIMBAL_PROTECTED || obs->valid == 0U)
    {
        Gimbal_ResetControl(gimbal);
        /* 目标跟随当前世界指向，上层以 gimbal->target 为增量基准，恢复时不跳变 */
        if (obs->valid != 0U)
        {
            gimbal->target.pitch_deg = obs->pitch_deg;
            gimbal->target.yaw_deg = obs->yaw_deg;
        }
        gimbal->target.pitch_omega_dps = 0.0f;
        gimbal->target.yaw_omega_dps = 0.0f;
        Gimbal_ApplyOutput(gimbal);
        return;
    }

    Gimbal_SolveJoint(gimbal);
    const GimbalJointRef_t *ref = &gimbal->controller.joint_ref;
    GimbalAxisCtrl_t *yaw = &gimbal->controller.axis[GIMBAL_YAW];
    GimbalAxisCtrl_t *pitch = &gimbal->controller.axis[GIMBAL_PITCH];
    if (RateDividerTick(&gimbal->controller.angle_loop) != 0U)
    {
        /* yaw 可整圈转，误差取最短路径；pitch 有限位，直接做差 */
        Gimbal_AngleLoop(yaw, AngleLimit(ref->angle_deg[GIMBAL_YAW] -
                                         obs->joint_deg[GIMBAL_YAW],
                                         -GIMBAL_YAW_WRAP_DEG, GIMBAL_YAW_WRAP_DEG));
        Gimbal_AngleLoop(pitch, ref->angle_deg[GIMBAL_PITCH] - obs->joint_deg[GIMBAL_PITCH]);
    }
    Gimbal_SpeedLoop(yaw, ref->omega_dps[GIMBAL_YAW], obs->joint_omega_dps[GIMBAL_YAW]);
    Gimbal_SpeedLoop(pitch, ref->omega_dps[GIMBAL_PITCH], obs->joint_omega_dps[GIMBAL_PITCH]);
    Gimbal_ApplyOutput(gimbal);
}

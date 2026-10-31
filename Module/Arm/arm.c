#include "arm.h"

#include <string.h>

#include "arm_jointctrl.h"
#include "arm_kinematics.h"

/* 重力算法按 ARM_GRAVITY_DOF 个关节写死维度，两处必须一致 */
_Static_assert(ARM_GRAVITY_DOF == ARM_JOINT_COUNT, "arm_dynamics.h 的关节数必须等于 ARM_JOINT_COUNT");

#define ARM_ENABLE_RESEND_DIV   200U    /* 使能态下使能帧重发间隔（Arm_Run 调用次数），防止首帧丢失 */

#define ARM_STATE_BIT(state)    (1U << (state))
#define ARM_MASTER_STATE_MASK   (ARM_STATE_BIT(ARM_PROTECTED) | ARM_STATE_BIT(ARM_ENABLE) | \
                                 ARM_STATE_BIT(ARM_FEEDBACK) | ARM_STATE_BIT(ARM_MASTER) | \
                                 ARM_STATE_BIT(ARM_HOST))
#define ARM_SERVANT_STATE_MASK  (ARM_STATE_BIT(ARM_PROTECTED) | ARM_STATE_BIT(ARM_ENABLE) | \
                                 ARM_STATE_BIT(ARM_AUTO) | ARM_STATE_BIT(ARM_SERVANT) | \
                                 ARM_STATE_BIT(ARM_HOST))

/* 目标状态是否属于本机角色的状态集 */
static bool Arm_StateAllowed(const Arm_t *arm, ArmState_t state)
{
    uint32_t mask = arm->config.state.role == ARM_ROLE_MASTER ? ARM_MASTER_STATE_MASK :
                                                                ARM_SERVANT_STATE_MASK;

    return (mask & ARM_STATE_BIT(state)) != 0U;
}

void Arm_Init(Arm_t *arm, const ArmConfig_t *config)
{
    memset(arm, 0, sizeof(*arm));
    arm->config = *config;
    arm->state = config->state.init_state;
    /* 无论初始状态是不是使能态，第一次进使能都要先对齐目标、清 PID */
    arm->ctrl_align_pending = 1U;
    arm->host_align_pending = config->state.init_state == ARM_HOST ? 1U : 0U;

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = config->motor.joint[i];
        const ArmJointGain_t *gain = &config->param.gain[i];

        /* 零点以模块参数为准，覆盖电机初始化时的默认值 */
        motor->zero_point = config->param.zero_point[i];
        /* 反馈帧绑到各自的 CAN 实例，回调只把数据解析进电机对象 */
        (void)Motor_BindCan(motor, config->periph.can[i]);
        /* 增益没填的关节保持全 0，输出恒为 0，即使能后不动 */
        PIDInitialize(&arm->joint_ctrl[i].pid, gain->kp, gain->ki, gain->kd,
                      gain->d_alpha, gain->sum_error_max, gain->output_max);
    }
}

/*--------------------------------------状态机--------------------------------------*/
bool Arm_SetState(Arm_t *arm, ArmState_t state)
{
    const ArmState_t previous_state = arm->state;

    if (!Arm_StateAllowed(arm, state))
    {
        return false;
    }

    if (state != arm->state)
    {
        arm->state = state;
        /* 进入新状态后第一个周期就发使能帧 */
        arm->enable_divider = 0U;
        /* 进使能态不能沿用上一个状态的目标和积分，下个周期先对齐观测 */
        arm->ctrl_align_pending = 1U;
        arm->host_align_pending = state == ARM_HOST ? 1U : 0U;

        /* 离开 HOST 或重新进入 HOST 都从零闭环、零有效目标开始 */
        arm->host_closed_loop_mask = 0U;
        arm->host_target_valid_mask = 0U;

        /* 从任何非保护状态切入保护时发一次失能；没有失能实现的电机跳过 */
        if (previous_state != ARM_PROTECTED && state == ARM_PROTECTED)
        {
            Arm_SendDisableCommand(arm);
        }
    }
    return true;
}

void Arm_SetRcOnline(Arm_t *arm, bool online)
{
    if (arm != NULL)
    {
        arm->rc_online = online ? 1U : 0U;
    }
}

ArmHostTargetResult_t Arm_SetHostTarget(Arm_t *arm, uint8_t joint, float target_abs_deg,
                                        bool closed_loop)
{
    const ArmJointLimit_t *limit;
    uint8_t bit;
    bool was_closed_loop;

    if (arm == NULL || joint >= ARM_JOINT_COUNT)
    {
        return ARM_HOST_TARGET_BAD_TARGET;
    }
    if (arm->state != ARM_HOST)
    {
        return ARM_HOST_TARGET_WRONG_STATE;
    }

    limit = &arm->config.param.limit[joint];
    if (!(target_abs_deg >= limit->min_deg && target_abs_deg <= limit->max_deg))
    {
        return ARM_HOST_TARGET_BAD_TARGET;
    }

    /* A target may arrive before the first HOST control cycle. Latch this
     * entry's limits first, then apply the PC target instead of letting the
     * deferred alignment overwrite it on the next Arm_Run. */
    if (arm->host_align_pending != 0U)
    {
        Arm_AlignHostTarget(arm);
        arm->host_align_pending = 0U;
        arm->ctrl_align_pending = 0U;
    }

    bit = (uint8_t)(1U << joint);
    was_closed_loop = (arm->host_closed_loop_mask & bit) != 0U;
    arm->host_target_abs_deg[joint] = target_abs_deg;
    arm->joint_ctrl[joint].target_deg = arm->observer.joint_deg[joint] +
                                        (target_abs_deg - arm->observer.joint_abs_deg[joint]);
    arm->host_target_valid_mask |= bit;
    if (closed_loop)
    {
        arm->host_closed_loop_mask |= bit;
    }
    else
    {
        arm->host_closed_loop_mask &= (uint8_t)~bit;
    }
    if (closed_loop != was_closed_loop)
    {
        PIDReset(&arm->joint_ctrl[joint].pid);
    }
    return ARM_HOST_TARGET_OK;
}

void Arm_StopHostControl(Arm_t *arm)
{
    if (arm == NULL)
    {
        return;
    }

    arm->host_closed_loop_mask = 0U;
    /* 只清上一周期已算好的前馈指令；Arm_Run 每周期都会重算，所以 HOST 下重力前馈会在下个周期恢复，
     * 要让它真正停掉靠切 ARM_PROTECTED（走零指令路径）或关闭 gravity.enable / 编译开关 */
    Arm_GravityClear(arm);
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        arm->joint_ctrl[i].output = 0.0f;
        PIDReset(&arm->joint_ctrl[i].pid);
    }
}

/*--------------------------------------重力补偿--------------------------------------*/
void Arm_GravityClear(Arm_t *arm)
{
    memset(arm->gravity.torque_cmd, 0, sizeof(arm->gravity.torque_cmd));
    arm->gravity.output_active = 0U;
}

void Arm_GravityUpdate(Arm_t *arm)
{
    ArmGravityState_t *gravity = &arm->gravity;

    gravity->model_valid = ArmDynamics_GravityModel(&arm->config.param.gravity, arm->config.param.dh,
                                                    arm->observer.joint_deg, gravity->torque_model) ? 1U : 0U;
    gravity->output_active = 0U;
    /* 重力只做符号和有效性处理，不做单轴预限幅；PID 与重力相加后由控制层统一做总力矩限幅 */
    ArmDynamics_GravityCommand(gravity->torque_model, gravity->model_valid != 0U,
                               arm->config.param.gravity.sign, NULL,
                               arm->config.param.gravity.joint_mask, gravity->torque_cmd);
}

void Arm_Run(Arm_t *arm)
{
    /*--------------------------------------观测--------------------------------------*/
    Arm_UpdateJointAngle(arm);
    Arm_ForwardKinematics(arm);
    Arm_ExtractPose(arm);
    /* 重力矩依赖本周期刚更新的关节角，所以放在观测之后、控制之前；只计算，是否输出由 ARM_GRAVITY_OUTPUT 决定 */
    Arm_GravityUpdate(arm);

    /*--------------------------------------控制与通信--------------------------------------*/
    /* 使能帧重发与目标来源解耦：ENABLE/HOST 都保持原来的周期重发节奏 */
    if (arm->state == ARM_ENABLE || arm->state == ARM_HOST)
    {
        if (arm->enable_divider == 0U)
        {
            Arm_SendEnableCommand(arm);
            arm->enable_divider = ARM_ENABLE_RESEND_DIV;
        }
        --arm->enable_divider;
    }

    Arm_UpdateTarget(arm);
    Arm_JointControl(arm);
}

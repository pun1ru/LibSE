#include "arm_jointctrl.h"

#include <stddef.h>
#include <string.h>

#include "algorism.h"

static void Arm_UpdateMultiTurnLimit(const Arm_t *arm, uint8_t joint,
                                     ArmJointCtrl_t *ctrl)
{
    const ArmJointLimit_t *limit = &arm->config.param.limit[joint];
    const float current_abs_deg = arm->observer.joint_abs_deg[joint];
    const float current_multi_deg = arm->observer.joint_deg[joint];

    ctrl->limit_min_deg = current_multi_deg + (limit->min_deg - current_abs_deg);
    ctrl->limit_max_deg = current_multi_deg + (limit->max_deg - current_abs_deg);
}

/*--------------------------------------控制--------------------------------------*/
/* 总力矩限幅：PID 与重力前馈各自限幅后相加，最终和再统一截到这里
 * total_output_max 未填或非正时最终输出为 0，避免没有配置却把不受限的力矩发出去 */
static float Arm_LimitTotalTorque(const Arm_t *arm, uint8_t joint, float torque)
{
    const float limit = arm->config.param.gain[joint].total_output_max;

    if (!(limit > 0.0f) || !((torque - torque) == 0.0f))
    {
        return 0.0f;
    }
    return torque > limit ? limit : (torque < -limit ? -limit : torque);
}

#if ARM_GRAVITY_OUTPUT
/* 闭环关节的最终输出 = PID 输出 + 重力前馈（sign 已在 Arm_GravityUpdate 里乘过，这里不再乘）
 * 重力前馈不受 PID 的 output_max 限制；两者的和由调用方统一做总力矩限幅 */
static float Arm_AddGravity(Arm_t *arm, uint8_t joint, float pid_output)
{
    if (arm->config.param.gravity.enable == 0U || arm->gravity.model_valid == 0U)
    {
        return pid_output;
    }
    arm->gravity.output_active = 1U;
    return pid_output + arm->gravity.torque_cmd[joint];
}
#endif

/* 进使能态的第一次控制前调用：锁存当前圈的多圈限位，再把目标对齐当前多圈关节角，PID 清零
 * 本次使能期间一直使用这组限位，跨圈后也不会重新映射到另一圈
 * 观测角在限位内时目标就等于观测角；不在限位内时截到最近边界 */
void Arm_AlignJointTarget(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];

        Arm_UpdateMultiTurnLimit(arm, i, ctrl);
        ctrl->target_deg = DoubleEdgeLimiter(arm->observer.joint_deg[i],
                                             ctrl->limit_min_deg,
                                             ctrl->limit_max_deg);
        ctrl->output = 0.0f;
        PIDReset(&ctrl->pid);
    }
}

void Arm_AlignHostTarget(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];

        Arm_UpdateMultiTurnLimit(arm, i, ctrl);
        ctrl->target_deg = DoubleEdgeLimiter(arm->observer.joint_deg[i],
                                             ctrl->limit_min_deg,
                                             ctrl->limit_max_deg);
        ctrl->output = 0.0f;
        PIDReset(&ctrl->pid);
        arm->host_target_abs_deg[i] = arm->observer.joint_abs_deg[i];
    }
    arm->host_closed_loop_mask = 0U;
    arm->host_target_valid_mask = 0U;
}

void Arm_TrackJointTarget(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        arm->joint_ctrl[i].target_deg = arm->observer.joint_deg[i];
    }
}

/* 按状态选择目标来源，限位也在这里收口：
 *   ENABLE：首次进入锁存限位、对齐观测并清 PID，之后目标持续跟随观测并被限位截断
 *   HOST：首次进入锁存限位、对齐观测并清 PID，之后目标来自上位机并被限位截断
 *   其他：目标跟随观测
 * 限位不在运行中重算，避免实际角跨圈后整个允许区间跟着跳到下一圈 */
void Arm_UpdateTarget(Arm_t *arm)
{
    switch (arm->state)
    {
    case ARM_ENABLE:
        if (arm->ctrl_align_pending != 0U)
        {
            Arm_AlignJointTarget(arm);
            arm->ctrl_align_pending = 0U;
        }
        for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
        {
            ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];

            ctrl->target_deg = DoubleEdgeLimiter(arm->observer.joint_deg[i],
                                                 ctrl->limit_min_deg,
                                                 ctrl->limit_max_deg);
        }
        break;

    case ARM_HOST:
        if (arm->host_align_pending != 0U)
        {
            Arm_AlignHostTarget(arm);
            arm->host_align_pending = 0U;
            arm->ctrl_align_pending = 0U;
        }
        for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
        {
            ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];

            ctrl->target_deg = DoubleEdgeLimiter(ctrl->target_deg,
                                                 ctrl->limit_min_deg,
                                                 ctrl->limit_max_deg);
        }
        break;

    default:
        Arm_TrackJointTarget(arm);
        break;
    }
}

/* 统一关节角度环：内部按状态选择闭环关节，不再区分普通控制和 HOST 控制
 *   ENABLE：全部关节 PID + 重力前馈
 *   HOST：host_closed_loop_mask 为 1 的关节 PID + 重力前馈；其余关节 PID 清零，只保留重力前馈
 *   其他：发零指令
 * 目标与限位已由 Arm_UpdateTarget 准备，这里不重复更新目标
 * 误差不做最短路径折叠；控制输出写进各电机并发送 */
void Arm_JointControl(Arm_t *arm)
{
    if (arm->state != ARM_ENABLE && arm->state != ARM_HOST)
    {
        Arm_SendZeroCommand(arm);
        return;
    }

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];
        const float observed_deg = arm->observer.joint_deg[i];
        const bool closed_loop =
            (arm->state == ARM_ENABLE) ||
            ((arm->host_closed_loop_mask & (uint8_t)(1U << i)) != 0U);

        if (closed_loop)
        {
            ctrl->output = PIDUpdate(&ctrl->pid, ctrl->target_deg - observed_deg);
        }
        else
        {
            ctrl->output = 0.0f;
            PIDReset(&ctrl->pid);
        }
#if ARM_GRAVITY_OUTPUT
        ctrl->output = Arm_AddGravity(arm, i, ctrl->output);
#endif
        ctrl->output = Arm_LimitTotalTorque(arm, i, ctrl->output);
    }
    Arm_SendJointCommand(arm);
}

/*--------------------------------------通信--------------------------------------*/
/* 按电机当前控制模式把力矩写到对应字段，不整体清零：位置、速度等字段保持驱动初始化后的值 */
static void Arm_WriteJointCommand(Motor_t *motor, float torque_nm)
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
        cmd->torque = torque_nm;
        break;
    case MODE_TORQUE:
        cmd->torque = torque_nm;
        break;
    case MODE_CURRENT:
        cmd->current = torque_nm;
        break;
    default:
        break;
    }
}

void Arm_SendZeroCommand(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];
        /* 指令整体清零，只保留初始化时定下的控制模式 */
        ControlMode_t mode = motor->motorcmd.control_mode;

        memset(&motor->motorcmd, 0, sizeof(motor->motorcmd));
        motor->motorcmd.control_mode = mode;
        motor->motorops.sendcommand(motor);
    }
}

void Arm_SendJointCommand(Arm_t *arm)
{
    /* 先写完所有关节再统一发送：同一 RM 电机组的关节共用一帧，边走边发会带出旧值 */
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Arm_WriteJointCommand(arm->config.motor.joint[i], arm->joint_ctrl[i].output);
    }
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];

        if (motor != NULL)
        {
            motor->motorops.sendcommand(motor);
        }
    }
}

void Arm_SendEnableCommand(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];

        if (motor->motorops.enable != NULL)
        {
            motor->motorops.enable(motor);
        }
    }
}

void Arm_SendDisableCommand(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];

        if (motor != NULL && motor->motorops.disable != NULL)
        {
            motor->motorops.disable(motor);
        }
    }
}

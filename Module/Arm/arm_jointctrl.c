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

void Arm_TrackJointTarget(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        arm->joint_ctrl[i].target_deg = arm->observer.joint_deg[i];
    }
}

/* 关节角度环：目标持续跟踪当前多圈观测，仅在离开进使能时锁存的限位区间时截断到边界
 * 限位不在运行中重算，避免实际角跨圈后整个允许区间跟着跳到下一圈 */
void Arm_JointControl(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];
        const float observed_deg = arm->observer.joint_deg[i];

        ctrl->target_deg = DoubleEdgeLimiter(observed_deg,
                                             ctrl->limit_min_deg,
                                             ctrl->limit_max_deg);
        ctrl->output = PIDUpdate(&ctrl->pid,
                                 ctrl->target_deg - observed_deg);
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

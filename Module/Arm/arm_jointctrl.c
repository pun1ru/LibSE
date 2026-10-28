#include "arm_jointctrl.h"

#include <stddef.h>
#include <string.h>

#include "algorism.h"

/*--------------------------------------控制--------------------------------------*/
/* 进使能态的第一次控制前调用：目标取当前关节角并截断到工作区间，PID 清零
 * 这样上电或保护恢复时不带旧目标，也不会因为积分残留突然出力 */
void Arm_AlignJointTarget(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];
        const ArmJointLimit_t *limit = &arm->config.param.limit[i];

        ctrl->target_deg = DoubleEdgeLimiter(arm->observer.joint_deg[i],
                                             limit->min_deg, limit->max_deg);
        ctrl->output = 0.0f;
        PIDReset(&ctrl->pid);
    }
}

/* 关节角度环：目标减观测，误差直接进 PID，输出即关节力矩
 * 不按最短路径折叠：限位区间连续且不跨 ±180，直接相减得到的正是限位内的那条路
 * 一旦折叠，跨度超过 180 的关节会走 ±180 那侧的禁行区（例如 [-150, 150] 里 -150 -> +150
 * 该走 +300，折叠成 -60 就顶在机械限位上死出力） */
void Arm_JointControl(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];
        const ArmJointLimit_t *limit = &arm->config.param.limit[i];

        /* 目标也可能由上层改动，每周期都按工作区间截断一次再进环 */
        ctrl->target_deg = DoubleEdgeLimiter(ctrl->target_deg, limit->min_deg, limit->max_deg);
        ctrl->output = PIDUpdate(&ctrl->pid,
                                 ctrl->target_deg - arm->observer.joint_deg[i]);
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

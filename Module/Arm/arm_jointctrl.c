#include "arm_jointctrl.h"

#include <stddef.h>
#include <string.h>

#include "algorism.h"
#include "arm_ik.h"

_Static_assert(ARM_TEST_CH4_RELEASE_THRESHOLD < ARM_TEST_CH4_SWITCH_THRESHOLD,
               "ARM_TEST CH4 release threshold must be below its switch threshold");

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
        arm->host.host_target_deg[i] = ctrl->target_deg;
    }
    arm->host.host_closed_loop_mask = 0U;
    arm->host.host_target_valid_mask = 0U;
}

void Arm_TrackJointTarget(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        arm->joint_ctrl[i].target_deg = arm->observer.joint_deg[i];
    }
}

static void Arm_UpdateTestTarget(Arm_t *arm)
{
    ArmTest_t *test = &arm->test;
    const int16_t ch1 = test->ch1;

    if (test->selected_joint >= ARM_JOINT_COUNT)
    {
        test->selected_joint = 0U;
    }

    /* CH4 is an edge selector. It cycles once per throw and must return to
     * the release band before another joint can be selected. */
    if (test->ch4 > ARM_TEST_CH4_SWITCH_THRESHOLD)
    {
        if (test->ch4_switch_armed != 0U)
        {
            test->selected_joint = (uint8_t)((test->selected_joint + 1U) % ARM_JOINT_COUNT);
            test->ch4_switch_armed = 0U;
        }
    }
    else if (test->ch4 < ARM_TEST_CH4_RELEASE_THRESHOLD)
    {
        test->ch4_switch_armed = 1U;
    }

    if (ch1 > ARM_TEST_CH1_DEADBAND)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[test->selected_joint];

        ctrl->target_deg += ((float)(ch1 - ARM_TEST_CH1_DEADBAND) /
                             (660.0f - (float)ARM_TEST_CH1_DEADBAND)) *
                            ARM_TEST_TARGET_MAX_STEP_DEG;
    }
    else if (ch1 < -ARM_TEST_CH1_DEADBAND)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[test->selected_joint];

        ctrl->target_deg += ((float)(ch1 + ARM_TEST_CH1_DEADBAND) /
                             (660.0f - (float)ARM_TEST_CH1_DEADBAND)) *
                            ARM_TEST_TARGET_MAX_STEP_DEG;
    }

    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[test->selected_joint];

        ctrl->target_deg = DoubleEdgeLimiter(ctrl->target_deg,
                                             ctrl->limit_min_deg,
                                             ctrl->limit_max_deg);
    }
}

/* 按状态选择目标来源，限位也在这里收口：
 *   ENABLE：首次进入锁存限位、对齐观测并清 PID，之后目标持续跟随观测并被限位截断
 *   HOST：首次进入锁存限位、对齐观测并清 PID，之后目标来自上位机并被限位截断
 *   TEST：首次进入锁存限位并对齐观测，之后 CH1 增量调整选中关节，CH4 循环切换关节
 *   其他：目标跟随观测
 * 限位不在运行中重算，避免实际角跨圈后整个允许区间跟着跳到下一圈 */
void Arm_UpdateTarget(Arm_t *arm)
{
    switch (arm->state)
    {
    case ARM_ENABLE:
        if (arm->host.ctrl_align_pending != 0U)
        {
            Arm_AlignJointTarget(arm);
            arm->host.ctrl_align_pending = 0U;
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
        if (arm->host.host_align_pending != 0U)
        {
            Arm_AlignHostTarget(arm);
            arm->host.host_align_pending = 0U;
            arm->host.ctrl_align_pending = 0U;
        }
        Arm_StepIk(arm);            /* 按配置的 IK 类型处理，再统一过限位 */
        for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
        {
            ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];

            ctrl->target_deg = DoubleEdgeLimiter(ctrl->target_deg,
                                                 ctrl->limit_min_deg,
                                                 ctrl->limit_max_deg);
        }
        break;

    case ARM_TEST:
        if (arm->test.align_pending != 0U)
        {
            Arm_AlignJointTarget(arm);
            arm->test.align_pending = 0U;
            arm->host.ctrl_align_pending = 0U;
            break;
        }
        Arm_UpdateTestTarget(arm);
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
 *   ENABLE：力矩轴 PID + 重力前馈，位置轴直接使用关节目标
 *   HOST：闭环力矩轴 PID + 重力前馈；开环轴只保留重力前馈
 *   TEST：仅由 CH1 修改当前选中关节目标，各轴按自身控制模式执行
 *   其他：发零指令
 * 目标与限位已由 Arm_UpdateTarget 准备，这里不重复更新目标
 * 误差不做最短路径折叠；控制输出写进各电机并发送 */
void Arm_JointControl(Arm_t *arm)
{
    if (arm->state != ARM_ENABLE && arm->state != ARM_HOST && arm->state != ARM_TEST)
    {
        Arm_SendZeroCommand(arm);
        return;
    }

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];
        Motor_t *motor = arm->config.motor.joint[i];
        const float observed_deg = arm->observer.joint_deg[i];
        const bool position_mode = motor->motorcmd.control_mode == MODE_POSITION;
        const bool closed_loop =
            (arm->state == ARM_ENABLE) || (arm->state == ARM_TEST) ||
            ((arm->host.host_closed_loop_mask & (uint8_t)(1U << i)) != 0U);

        if (position_mode)
        {
            /* The LK driver owns the position loop; do not stack the Arm PID on it. */
            ctrl->output = 0.0f;
            PIDReset(&ctrl->pid);
        }
        else if (closed_loop)
        {
            ctrl->output = PIDUpdate(&ctrl->pid, ctrl->target_deg - observed_deg);
        }
        else
        {
            ctrl->output = 0.0f;
            PIDReset(&ctrl->pid);
        }
#if ARM_GRAVITY_OUTPUT
        if (!position_mode)
        {
            ctrl->output = Arm_AddGravity(arm, i, ctrl->output);
        }
#endif
        if (!position_mode)
        {
            ctrl->output = Arm_LimitTotalTorque(arm, i, ctrl->output);
        }
    }
    Arm_SendJointCommand(arm);
}

/*--------------------------------------通信--------------------------------------*/
/* 按电机当前控制模式把关节目标和力矩写到对应字段，不整体清零。 */
static void Arm_WriteJointCommand(const Arm_t *arm, uint8_t joint, bool active)
{
    Motor_t *motor = arm->config.motor.joint[joint];

    if (motor == NULL)
    {
        return;
    }

    MotorCmd_t *cmd = &motor->motorcmd;
    switch (cmd->control_mode)
    {
    case MODE_POSITION:
        if (active)
        {
            const float direction =
                arm->config.param.direction[joint] < 0 ? -1.0f : 1.0f;

            cmd->position = direction * arm->joint_ctrl[joint].target_deg *
                            MOTOR_DEG_TO_RAD;
        }
        break;
    case MODE_MIT:
        cmd->position = 0.0f;
        cmd->velocity = 0.0f;
        cmd->kp = 0.0f;
        cmd->kd = 0.0f;
        cmd->torque = arm->joint_ctrl[joint].output;
        break;
    case MODE_TORQUE:
        cmd->torque = arm->joint_ctrl[joint].output;
        break;
    case MODE_CURRENT:
        cmd->current = arm->joint_ctrl[joint].output;
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
        if (motor->motorcmd.control_mode == MODE_POSITION)
        {
            /* Do not stream 0x81 in protection: it causes LK nodes to stop
             * answering 0x92 between Stop and Run. */
            if (motor->motorops.requestfeedback != NULL)
            {
                motor->motorops.requestfeedback(motor);
            }
            continue;
        }
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
        const bool active =
            arm->state == ARM_ENABLE || arm->state == ARM_TEST ||
            (arm->state == ARM_HOST &&
             (arm->host.host_closed_loop_mask & (uint8_t)(1U << i)) != 0U);

        Arm_WriteJointCommand(arm, i, active);
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

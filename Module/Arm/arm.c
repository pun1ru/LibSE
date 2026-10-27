#include "arm.h"

#include <string.h>

#include "arm_jointctrl.h"
#include "arm_kinematics.h"

#define ARM_ENABLE_RESEND_DIV   200U    /* 使能态下使能帧重发间隔（Arm_Run 调用次数），防止首帧丢失 */

#define ARM_STATE_BIT(state)    (1U << (state))
#define ARM_MASTER_STATE_MASK   (ARM_STATE_BIT(ARM_PROTECTED) | ARM_STATE_BIT(ARM_ENABLE) | \
                                 ARM_STATE_BIT(ARM_FEEDBACK) | ARM_STATE_BIT(ARM_MASTER))
#define ARM_SERVANT_STATE_MASK  (ARM_STATE_BIT(ARM_PROTECTED) | ARM_STATE_BIT(ARM_ENABLE) | \
                                 ARM_STATE_BIT(ARM_AUTO) | ARM_STATE_BIT(ARM_SERVANT))

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

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = config->motor.joint[i];

        /* 零点以模块参数为准，覆盖电机初始化时的默认值 */
        motor->zero_point = config->param.zero_point[i];
        /* 反馈帧绑到各自的 CAN 实例，回调只把数据解析进电机对象 */
        (void)Motor_BindCan(motor, config->periph.can[i]);
    }
}

/*--------------------------------------状态机--------------------------------------*/
bool Arm_SetState(Arm_t *arm, ArmState_t state)
{
    if (!Arm_StateAllowed(arm, state))
    {
        return false;
    }

    if (state != arm->state)
    {
        arm->state = state;
        /* 进入新状态后第一个周期就发使能帧 */
        arm->enable_divider = 0U;
    }
    return true;
}

void Arm_Run(Arm_t *arm)
{
    /*--------------------------------------观测--------------------------------------*/
    Arm_UpdateJointAngle(arm);
    Arm_ForwardKinematics(arm);
    Arm_ExtractPose(arm);

    /*--------------------------------------控制与通信--------------------------------------*/
    switch (arm->state)
    {
    case ARM_ENABLE:
        if (arm->enable_divider == 0U)
        {
            Arm_SendEnableCommand(arm);
            arm->enable_divider = ARM_ENABLE_RESEND_DIV;
        }
        --arm->enable_divider;
        Arm_SendZeroCommand(arm);
        break;
    /* FEEDBACK / MASTER / AUTO / SERVANT 的关节控制还没实现，先和保护一样发零指令 */
    case ARM_PROTECTED:
    default:
        Arm_SendZeroCommand(arm);
        break;
    }
}

#include "arm.h"

#include <string.h>

void Arm_Init(Arm_t *arm, const ArmConfig_t *config)
{
    memset(arm, 0, sizeof(*arm));
    arm->config = *config;
    arm->state = config->state.init_state;

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        /* 反馈帧绑到各自的 CAN 实例，回调只把数据解析进电机对象 */
        (void)Motor_BindCan(config->motor.joint[i], config->periph.can[i]);
    }
}

/*--------------------------------------通信--------------------------------------*/
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

void Arm_Run(Arm_t *arm)
{
    switch (arm->state)
    {
    case ARM_PROTECTED:
    default:
        Arm_SendZeroCommand(arm);
        break;
    }
}

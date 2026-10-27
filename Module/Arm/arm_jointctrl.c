#include "arm_jointctrl.h"

#include <stddef.h>
#include <string.h>

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

#include "arm_kinematics.h"

#include "position_tran.h"

/*--------------------------------------零点标定--------------------------------------*/
bool Arm_CalibrateZero(Arm_t *arm)
{
    /* 没收到过反馈的电机 encoder 还是初值，用它标定会得到错误零点，先全部检查再写 */
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        if (arm->config.motor.joint[i]->feedback.status == MOTOR_NO_FEEDBACK)
        {
            return false;
        }
    }

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];

        motor->zero_point = motor->feedback.encoder;
        arm->config.param.zero_point[i] = motor->zero_point;
    }
    return true;
}

/*--------------------------------------正运动学--------------------------------------*/
void Arm_UpdateJointAngle(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        const Motor_t *motor = arm->config.motor.joint[i];
        /* 标定时没填（0）按 +1 处理，避免忘记配置时关节角恒为 0 */
        const int8_t sign = arm->config.param.direction[i];

        arm->observer.joint_deg[i] = (sign < 0 ? -1.0f : 1.0f) * motor->feedback.abs_angle_deg;
    }
}

void Arm_ForwardKinematics(Arm_t *arm)
{
    DH_ForwardKinematics(arm->config.param.dh, arm->observer.joint_deg, ARM_JOINT_COUNT,
                         arm->observer.end_to_base);
}

void Arm_ExtractPose(Arm_t *arm)
{
    ArmObserver_t *obs = &arm->observer;
    float rotation[3][3];
    float euler_rad[ARM_VECTOR_DIM];

    Trans4_GetPos(obs->end_to_base, obs->position_m);
    Trans4_GetRotation(obs->end_to_base, rotation);
    Matrix_ToEuler(rotation, euler_rad);
    for (uint8_t i = 0U; i < ARM_VECTOR_DIM; ++i)
    {
        obs->euler_deg[i] = euler_rad[i] * MOTOR_RAD_TO_DEG;
    }
}

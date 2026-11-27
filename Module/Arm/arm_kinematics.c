#include "arm_kinematics.h"

#include <math.h>
#include <string.h>

#include "position_tran.h"

/* DH 末端 -> TCP。TCP 原点位于 W 点沿末端 z 轴反向 0.045 m；
 * x 轴取末端 z 轴，y 轴取 J5 零位方向（末端 y 轴），z 轴由右手系 X×Y 确定。 */
static const float s_end_to_tcp[4][4] = {
    {0.0f, 0.0f, -1.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f, -0.045f},
    {0.0f, 0.0f, 0.0f, 1.0f},
};

static void Arm_RigidInverse(const float in[4][4], float out[4][4])
{
    for (uint8_t row = 0U; row < 3U; ++row)
    {
        for (uint8_t column = 0U; column < 3U; ++column)
        {
            out[row][column] = in[column][row];
        }
        out[row][3] = -(out[row][0] * in[0][3] +
                        out[row][1] * in[1][3] +
                        out[row][2] * in[2][3]);
    }
    out[3][0] = 0.0f;
    out[3][1] = 0.0f;
    out[3][2] = 0.0f;
    out[3][3] = 1.0f;
}

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
        Motor_t *motor = arm->config.motor.joint[i];
        /* 标定时没填（0）按 +1 处理，避免忘记配置时关节角恒为 0 */
        const int8_t sign = arm->config.param.direction[i];
        const float direction = sign < 0 ? -1.0f : 1.0f;

        Motor_UpdateFeedbackStatus(motor);
        arm->observer.joint_abs_deg[i] = direction * motor->feedback.abs_angle_deg;
        arm->observer.joint_deg[i] = direction * motor->feedback.multi_angle_deg;
    }
}

void Arm_ForwardKinematics(Arm_t *arm)
{
    DH_ForwardKinematics(arm->config.param.dh, arm->observer.joint_deg, ARM_JOINT_COUNT,
                         arm->observer.end_to_base);
    Trans4_Mul(arm->observer.end_to_base, s_end_to_tcp,
               arm->observer.tcp_to_base);
}

void Arm_ExtractPose(Arm_t *arm)
{
    ArmObserver_t *obs = &arm->observer;
    float rotation[3][3];
    float euler_rad[ARM_VECTOR_DIM];

    Trans4_GetPos(obs->tcp_to_base, obs->position_m);
    Trans4_GetRotation(obs->tcp_to_base, rotation);
    Matrix_ToEuler(rotation, euler_rad);
    for (uint8_t i = 0U; i < ARM_VECTOR_DIM; ++i)
    {
        obs->euler_deg[i] = euler_rad[i] * MOTOR_RAD_TO_DEG;
    }

    memcpy(obs->tcp_position_m, obs->position_m, sizeof(obs->tcp_position_m));
    memcpy(obs->tcp_euler_deg, obs->euler_deg, sizeof(obs->tcp_euler_deg));
}

void Arm_TcpTargetToEnd(const float tcp_target[4][4], float end_target[4][4])
{
    float tcp_to_end[4][4];

    Arm_RigidInverse(s_end_to_tcp, tcp_to_end);
    Trans4_Mul(tcp_target, tcp_to_end, end_target);
}

bool Arm_GetIkTargetTcpPose(const Arm_t *arm, float position_m[3], float euler_deg[3])
{
    float tcp_target[4][4];
    float rotation[3][3];
    float euler_rad[3];

    if (arm == NULL || position_m == NULL || euler_deg == NULL ||
        arm->ik.active == 0U)
    {
        return false;
    }
    Trans4_Mul(arm->ik.target, s_end_to_tcp, tcp_target);
    Trans4_GetPos(tcp_target, position_m);
    Trans4_GetRotation(tcp_target, rotation);
    Matrix_ToEuler(rotation, euler_rad);
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        euler_deg[i] = euler_rad[i] * MOTOR_RAD_TO_DEG;
        if (!isfinite(position_m[i]) || !isfinite(euler_deg[i]))
        {
            return false;
        }
    }
    return true;
}

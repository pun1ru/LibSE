/**
 * @file    MadWick.c
 * @brief   Madgwick AHRS 姿态解算实现
 * @note    纯 C 移植自 MadgwickAHRS.hpp
 *          核心：陀螺仪积分 + 梯度下降法加速度修正
 *          复用 algorism.h 的 InvSqrt()
 */

#include "MadWick.h"
#include "algorism.h"
#include <math.h>

/*---------------------------------------------------------------------------初始化-------------------------------------------------------------------------------------------*/

/**
 * @brief   初始化 Madgwick AHRS
 * @param   m    结构体指针
 * @param   beta 梯度下降增益
 * @retval  void
 * @note    四元数初始化为 [1, 0, 0, 0]（无旋转），欧拉角清零
 */
void MadWickAHRSInit(MadWickAHRS* m, float beta)
{
    m->q[0] = 1.0f;
    m->q[1] = 0.0f;
    m->q[2] = 0.0f;
    m->q[3] = 0.0f;
    m->beta  = beta;
    m->Pitch = 0.0f;
    m->Roll  = 0.0f;
    m->Yaw   = 0.0f;
}

/*---------------------------------------------------------------------------姿态更新-----------------------------------------------------------------------------------------*/

/**
 * @brief   Madgwick AHRS 姿态更新
 * @param   m  结构体指针
 * @param   gx/gy/gz 角速度 (rad/s, 已减零偏)
 * @param   ax/ay/az 加速度 (m/s²)
 * @param   dt 更新周期 (s)
 * @retval  void
 * @note    算法步骤：
 *          1. 陀螺仪积分 → 四元数变化率 q_dot
 *          2. 加速度计梯度下降 → 修正步长 s0~s3
 *          3. q_dot -= beta * s   （梯度修正）
 *          4. q += q_dot * dt     （欧拉积分）
 *          5. 四元数归一化
 *          6. 四元数 → 欧拉角
 */
void MadWickAHRSUpdate(MadWickAHRS* m,
                       float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt)
{
    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;
    float q_2q0, q_2q1, q_2q2, q_2q3, q_4q0, q_4q1, q_4q2, q_8q1, q_8q2;
    float q0q0, q1q1, q2q2, q3q3;

    /* 局部变量减少结构体成员多次访问 */
    float q0 = m->q[0];
    float q1 = m->q[1];
    float q2 = m->q[2];
    float q3 = m->q[3];

    /* ===== Step 1: 陀螺仪积分 → 四元数变化率 ===== */
    /* q_dot = 0.5 * q ⊗ ω */
    q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    q_dot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    q_dot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    /* ===== Step 2: 加速度计梯度下降修正 ===== */
    /* 仅在加速度计有效时进行（避免除以零） */
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        /* 归一化加速度测量值 */
        recip_norm = InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* 辅助变量，减少重复计算 */
        q_2q0 = 2.0f * q0;
        q_2q1 = 2.0f * q1;
        q_2q2 = 2.0f * q2;
        q_2q3 = 2.0f * q3;
        q_4q0 = 4.0f * q0;
        q_4q1 = 4.0f * q1;
        q_4q2 = 4.0f * q2;
        q_8q1 = 8.0f * q1;
        q_8q2 = 8.0f * q2;
        q0q0  = q0 * q0;
        q1q1  = q1 * q1;
        q2q2  = q2 * q2;
        q3q3  = q3 * q3;

        /* 梯度下降算法修正步长 */
        s0 = q_4q0 * q2q2 + q_2q2 * ax + q_4q0 * q1q1 - q_2q1 * ay;
        s1 = q_4q1 * q3q3 - q_2q3 * ax + 4.0f * q0q0 * q1 - q_2q0 * ay
           - q_4q1 + q_8q1 * q1q1 + q_8q1 * q2q2 + q_4q1 * az;
        s2 = 4.0f * q0q0 * q2 + q_2q0 * ax + q_4q2 * q3q3 - q_2q3 * ay
           - q_4q2 + q_8q2 * q1q1 + q_8q2 * q2q2 + q_4q2 * az;
        s3 = 4.0f * q1q1 * q3 - q_2q1 * ax + 4.0f * q2q2 * q3 - q_2q2 * ay;

        /* 归一化步长 */
        recip_norm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        /* ===== Step 3: 反馈修正 ===== */
        /* q_dot -= beta * s  (梯度下降方向修正陀螺积分) */
        q_dot1 -= m->beta * s0;
        q_dot2 -= m->beta * s1;
        q_dot3 -= m->beta * s2;
        q_dot4 -= m->beta * s3;
    }

    /* ===== Step 4: 欧拉积分 ===== */
    q0 += q_dot1 * dt;
    q1 += q_dot2 * dt;
    q2 += q_dot3 * dt;
    q3 += q_dot4 * dt;

    /* ===== Step 5: 四元数归一化 ===== */
    recip_norm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    m->q[0] = q0 * recip_norm;
    m->q[1] = q1 * recip_norm;
    m->q[2] = q2 * recip_norm;
    m->q[3] = q3 * recip_norm;

    /* ===== Step 6: 四元数 → 欧拉角 (ZYX, deg) ===== */
    /* 与 ekf_quaternion.c 采用相同转换公式 */
    m->Yaw   = atan2f(2.0f * (m->q[0] * m->q[3] + m->q[1] * m->q[2]),
                      2.0f * (m->q[0] * m->q[0] + m->q[1] * m->q[1]) - 1.0f)
             * 57.295779513f;

    m->Pitch = atan2f(2.0f * (m->q[0] * m->q[1] + m->q[2] * m->q[3]),
                      2.0f * (m->q[0] * m->q[0] + m->q[3] * m->q[3]) - 1.0f)
             * 57.295779513f;

    m->Roll  = asinf(-2.0f * (m->q[1] * m->q[3] - m->q[0] * m->q[2]))
             * 57.295779513f;
}

#ifndef POSITION_TRAN_H
#define POSITION_TRAN_H

#include <math.h>

/* 四元数、欧拉角、旋转矩阵相互转换，输入输出都是数组
 * 四元数   q[4]      = [w, x, y, z]，与 Madgwick / ekf_quaternion 一致
 * 欧拉角   euler[3]  = [roll, pitch, yaw]，单位 rad，分别绕 X / Y / Z 轴
 *                      按 Z-Y-X 顺序：R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * 旋转矩阵 R[3][3]   行优先，子坐标系向量变换到父坐标系：v_parent = R * v_child */

#define EULER_ROLL   0U
#define EULER_PITCH  1U
#define EULER_YAW    2U

#define POSITION_TRAN_LOCK_EPS  1e-6f   /* cos(pitch) 低于此值视为万向节死锁 */

/* 3x3 / 3 维向量运算，out 不能与输入别名
 * 旋转矩阵正交，逆即转置，所以只提供带转置的乘法，不做通用求逆 */

/* out = A * B */
static inline void Mat3_Mul(const float A[3][3], const float B[3][3], float out[3][3])
{
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        for (unsigned int j = 0U; j < 3U; ++j)
        {
            out[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
        }
    }
}

/* out = A * B^T */
static inline void Mat3_MulT(const float A[3][3], const float B[3][3], float out[3][3])
{
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        for (unsigned int j = 0U; j < 3U; ++j)
        {
            out[i][j] = A[i][0] * B[j][0] + A[i][1] * B[j][1] + A[i][2] * B[j][2];
        }
    }
}

/* out = A * v */
static inline void Mat3_MulVec(const float A[3][3], const float v[3], float out[3])
{
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        out[i] = A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2];
    }
}

/* out = A^T * v */
static inline void Mat3_TransMulVec(const float A[3][3], const float v[3], float out[3])
{
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        out[i] = A[0][i] * v[0] + A[1][i] * v[1] + A[2][i] * v[2];
    }
}

/* 四元数 -> 旋转矩阵，按模长平方归一化，输入不必严格是单位四元数 */
static inline void Quat_ToMatrix(const float q[4], float R[3][3])
{
    float n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    float s = n > 0.0f ? 2.0f / n : 0.0f;
    float wx = s * q[0] * q[1], wy = s * q[0] * q[2], wz = s * q[0] * q[3];
    float xx = s * q[1] * q[1], xy = s * q[1] * q[2], xz = s * q[1] * q[3];
    float yy = s * q[2] * q[2], yz = s * q[2] * q[3], zz = s * q[3] * q[3];

    R[0][0] = 1.0f - (yy + zz);
    R[0][1] = xy - wz;
    R[0][2] = xz + wy;
    R[1][0] = xy + wz;
    R[1][1] = 1.0f - (xx + zz);
    R[1][2] = yz - wx;
    R[2][0] = xz - wy;
    R[2][1] = yz + wx;
    R[2][2] = 1.0f - (xx + yy);
}

/* 欧拉角 -> 旋转矩阵，Rz(yaw) * Ry(pitch) * Rx(roll) 展开 */
static inline void Euler_ToMatrix(const float euler[3], float R[3][3])
{
    float cr = cosf(euler[EULER_ROLL]),  sr = sinf(euler[EULER_ROLL]);
    float cp = cosf(euler[EULER_PITCH]), sp = sinf(euler[EULER_PITCH]);
    float cy = cosf(euler[EULER_YAW]),   sy = sinf(euler[EULER_YAW]);

    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}
/* 欧拉角 -> 四元数 */
static inline void Euler_ToQuat(const float euler[3], float q[4])
{
    float cr = cosf(0.5f * euler[EULER_ROLL]),  sr = sinf(0.5f * euler[EULER_ROLL]);
    float cp = cosf(0.5f * euler[EULER_PITCH]), sp = sinf(0.5f * euler[EULER_PITCH]);
    float cy = cosf(0.5f * euler[EULER_YAW]),   sy = sinf(0.5f * euler[EULER_YAW]);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

/* 旋转矩阵 -> 欧拉角，pitch 范围 [-pi/2, pi/2]
 * 万向节死锁时 roll 与 yaw 不可分，约定 yaw = 0，转角全部归到 roll */
static inline void Matrix_ToEuler(const float R[3][3], float euler[3])
{
    float horizon = sqrtf(R[0][0] * R[0][0] + R[1][0] * R[1][0]);

    euler[EULER_PITCH] = atan2f(-R[2][0], horizon);
    if (horizon > POSITION_TRAN_LOCK_EPS)
    {
        euler[EULER_ROLL] = atan2f(R[2][1], R[2][2]);
        euler[EULER_YAW] = atan2f(R[1][0], R[0][0]);
    }
    else
    {
        euler[EULER_ROLL] = atan2f(-R[1][2], R[1][1]);
        euler[EULER_YAW] = 0.0f;
    }
}

/* 旋转矩阵 -> 四元数，按最大对角分量选分支（Shepperd 法），输出 w >= 0 */
static inline void Matrix_ToQuat(const float R[3][3], float q[4])
{
    float trace = R[0][0] + R[1][1] + R[2][2];
    float s;

    if (trace > 0.0f)
    {
        s = 2.0f * sqrtf(1.0f + trace);
        q[0] = 0.25f * s;
        q[1] = (R[2][1] - R[1][2]) / s;
        q[2] = (R[0][2] - R[2][0]) / s;
        q[3] = (R[1][0] - R[0][1]) / s;
    }
    else if (R[0][0] > R[1][1] && R[0][0] > R[2][2])
    {
        s = 2.0f * sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]);
        q[0] = (R[2][1] - R[1][2]) / s;
        q[1] = 0.25f * s;
        q[2] = (R[0][1] + R[1][0]) / s;
        q[3] = (R[0][2] + R[2][0]) / s;
    }
    else if (R[1][1] > R[2][2])
    {
        s = 2.0f * sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]);
        q[0] = (R[0][2] - R[2][0]) / s;
        q[1] = (R[0][1] + R[1][0]) / s;
        q[2] = 0.25f * s;
        q[3] = (R[1][2] + R[2][1]) / s;
    }
    else
    {
        s = 2.0f * sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]);
        q[0] = (R[1][0] - R[0][1]) / s;
        q[1] = (R[0][2] + R[2][0]) / s;
        q[2] = (R[1][2] + R[2][1]) / s;
        q[3] = 0.25f * s;
    }

    float n = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (q[0] < 0.0f)
    {
        n = -n;
    }
    for (unsigned int i = 0U; i < 4U; ++i)
    {
        q[i] *= n;
    }
}

/* 四元数 -> 欧拉角，经旋转矩阵中转，死锁处理与 Matrix_ToEuler 一致 */
static inline void Quat_ToEuler(const float q[4], float euler[3])
{
    float R[3][3];
    Quat_ToMatrix(q, R);
    Matrix_ToEuler((const float (*)[3])R, euler);
}

#endif

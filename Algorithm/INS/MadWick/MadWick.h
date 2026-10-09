/**
 * @file    MadWick.h
 * @brief   Madgwick AHRS 姿态解算 —— 梯度下降法融合陀螺仪+加速度计
 * @note    移植自 MadgwickAHRS.hpp (C++ LibXR 框架) → 纯 C
 *          复用 algorism.h 的 InvSqrt()
 *          四元数顺序 [w, x, y, z]，与 ekf_quaternion.h 一致
 */

#ifndef _MADWICK_AHRS_H_
#define _MADWICK_AHRS_H_

#include <stdint.h>

/**
 * @brief Madgwick AHRS 结构体
 * @note  beta 是唯一调参参数：
 *        - 大 (0.1~0.5): 加速度计权重大，抗陀螺漂移强，但噪声大
 *        - 小 (0.01~0.05): 更信陀螺积分，动态好，但yaw会漂
 *        - 典型值: 0.05（默认），磁力计辅助时可用更小的beta
 */
typedef struct
{
    float q[4];          /* 四元数 [w, x, y, z] */
    float beta;          /* Madgwick 梯度下降增益 */
    float Pitch;         /* 俯仰角 (deg)  抬头为正 */
    float Roll;          /* 横滚角 (deg)  右侧下沉为正 */
    float Yaw;           /* 偏航角 (deg)  顺时针为正 */
} MadWickAHRS;

/**
 * @brief   初始化 Madgwick AHRS
 * @param   m    结构体指针
 * @param   beta 梯度下降增益，典型值 0.05
 * @retval  void
 */
void MadWickAHRSInit(MadWickAHRS* m, float beta);

/**
 * @brief   Madgwick AHRS 姿态更新（每周期调用一次）
 * @param   m  结构体指针
 * @param   gx gyro X (rad/s) — 绕机体x轴
 * @param   gy gyro Y (rad/s) — 绕机体y轴
 * @param   gz gyro Z (rad/s) — 绕机体z轴
 * @param   ax accel X (m/s²) — 机体x轴加速度
 * @param   ay accel Y (m/s²) — 机体y轴加速度
 * @param   az accel Z (m/s²) — 机体z轴加速度
 * @param   dt 姿态更新周期 (s)
 * @retval  void
 * @note    gx/gy/gz 应为减过零偏后的角速度
 *          调用后 m->q[4]、m->Yaw/Pitch/Roll 被更新
 */
void MadWickAHRSUpdate(MadWickAHRS* m,
                       float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt);

#endif /* _MADWICK_AHRS_H_ */

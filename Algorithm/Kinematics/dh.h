#ifndef DH_H
#define DH_H

#include <stdint.h>

/**
 * \brief 标准 DH 参数（一行 = 一个关节，连杆 i 相对 i-1）
 * \note  T_i = Rz(q + theta_offset) * Tz(d) * Tx(a) * Rx(alpha)
 */
typedef struct {
    float theta_offset_rad; ///< 关节角零位偏置
    float d_m;              ///< 沿 z_{i-1} 平移
    float a_m;              ///< 沿 x_i 平移
    float alpha_rad;        ///< 绕 x_i 旋转
} DH_Param_t;

/**
 * \brief 单个关节的齐次变换矩阵
 * \param[in]  param DH 参数
 * \param[in]  q_rad 关节角
 * \param[out] T     4x4 齐次变换（行优先）
 */
void DH_Transform(const DH_Param_t *param, float q_rad, float T[4][4]);

/**
 * \brief 按 DH 参数表做正运动学 T = T_1 * T_2 * ... * T_n
 * \param[in]  table DH 参数表
 * \param[in]  q_rad 各关节角，长度 n
 * \param[in]  n     关节数
 * \param[out] T     基座到末端的 4x4 齐次变换
 */
void DH_ForwardKinematics(const DH_Param_t *table, const float *q_rad,
                          uint8_t n, float T[4][4]);

#endif

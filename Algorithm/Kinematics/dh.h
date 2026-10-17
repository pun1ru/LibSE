#ifndef DH_H
#define DH_H

#include <stdint.h>

/**
 * \brief 改进 DH（Craig）参数，一行 = 坐标系 i 相对 i-1
 * \note  T_{i-1,i} = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(q_i + theta_offset_i) * Tz(d_i)，角度单位 deg
 *        字段顺序与参数表列 | αi-1 | ai-1 | di | θi | 一致；末端固定坐标系那一行 q 恒为 0
 */
typedef struct {
    float alpha_deg;        ///< αi-1：绕 x_{i-1} 旋转
    float a_m;              ///< ai-1：沿 x_{i-1} 平移
    float d_m;              ///< di：沿 z_i 平移
    float theta_offset_deg; ///< θi 的零位偏置，实际 θi = q_i + theta_offset
} DH_Param_t;

/**
 * \brief 单行的齐次变换矩阵
 * \param[in]  param DH 参数
 * \param[in]  q_deg 关节角
 * \param[out] T     4x4 齐次变换（行优先）
 */
void DH_Transform(const DH_Param_t *param, float q_deg, float T[4][4]);

/**
 * \brief 按 DH 参数表做正运动学 T = T_01 * T_12 * ... * T_{n-1,n}
 * \param[in]  table DH 参数表
 * \param[in]  q_deg 各行关节角，长度 n
 * \param[in]  n     行数
 * \param[out] T     基座到第 n 个坐标系的 4x4 齐次变换
 */
void DH_ForwardKinematics(const DH_Param_t *table, const float *q_deg,
                          uint8_t n, float T[4][4]);

#endif

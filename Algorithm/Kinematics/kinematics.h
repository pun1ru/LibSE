#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

#include "position_tran.h"

/* 旋转关节的速度在相邻连杆坐标系之间的转换，单位 m/s、rad/s
 * T 为 DH 给出的 4x4 齐次变换，子坐标系 -> 父坐标系（{i+1} -> {i}），可由 DH_Transform 得到
 * 速度、角速度各自表示在所在连杆自己的坐标系下，linear 是坐标系原点处的速度
 * 只处理旋转关节，关节轴为 z 轴
 * 输出可以与输入是同一个对象，便于沿连杆原地递推 */

#define KIN_AXIS_Z  2U

typedef struct {
    float linear[3];
    float angular[3];
} Twist_t;

/* 子坐标系的速度 -> 父坐标系，刚体上同一点换到父系原点处：v_p = R v_c + p x w_p */
void Kin_TwistToParent(const float T[4][4], const Twist_t *child, Twist_t *parent);

/* 父坐标系的速度 -> 子坐标系，不含关节自身转速：w_c = R^T w_p，v_c = R^T (v_p + w_p x p) */
void Kin_TwistToChild(const float T[4][4], const Twist_t *parent, Twist_t *child);

/* 连杆外推：{i} 的速度加上关节 i+1 的转速 joint_rate（rad/s），得到 {i+1} 的速度
 * 角速度在 Kin_TwistToChild 的结果上沿 z 轴加 joint_rate */
void Kin_PropagateRevolute(const float T[4][4], const Twist_t *parent, float joint_rate,
                           Twist_t *child);

/* 角速度的传递：{i} 系下的角速度 -> {i+1} 系下，w_c = R^T w_p + 关节转速沿 z 轴，不涉及线速度
 * 输出可以与输入是同一个对象 */
void Kin_PropagateAngular(const float T[4][4], const float parent[3], float joint_rate,
                          float child[3]);

/* 关节链：T[i] 为 {i+1} -> {i}（DH_LinkTransforms 的输出），rate[i] 为关节 i+1 的转速（rad/s），n 为关节数
 * 输出数组长度 n + 1，out[0] 为基座，out[i] 表示在 {i} 系下 */
void Kin_ChainAngular(const float (*T)[4][4], const float *rate, uint8_t n, const float base[3],
                      float (*out)[3]);
void Kin_ChainTwist(const float (*T)[4][4], const float *rate, uint8_t n, const Twist_t *base,
                    Twist_t *out);

/* 角速度都表示在基座系下：w^0_i = w^0_{i-1} + rate_i * z_i，z_i 是 {i} 系 z 轴在基座系下的方向
 * 输出数组长度 n + 1，out[0] 为基座角速度 base（基座系下） */
void Kin_ChainAngularInBase(const float (*T)[4][4], const float *rate, uint8_t n,
                            const float base[3], float (*out)[3]);

#endif

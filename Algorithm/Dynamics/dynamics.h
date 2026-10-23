#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <stdint.h>

#include "kinematics.h"

/* 旋转关节的加速度、力、力矩、转动惯量在相邻连杆坐标系之间的转换，SI 单位
 * T 与 kinematics.h 相同：DH 给出的 4x4 齐次变换，子坐标系 -> 父坐标系（{i+1} -> {i}）
 * 量都表示在各自连杆自己的坐标系下，关节轴为 z 轴，递推式采用牛顿欧拉法
 * 重力通过基座的线加速度给出：基座 accel.linear 取竖直向上的 g，即等效于整个机构向上加速 */

typedef struct {
    float linear[3];
    float angular[3];
} Accel_t;

typedef struct {
    Twist_t twist;
    Accel_t accel;
} LinkMotion_t;

typedef struct {
    float force[3];
    float moment[3];
} Wrench_t;

/* 连杆惯性参数，质心位置与惯量都表示在连杆自己的坐标系下，惯量是绕质心的 */
typedef struct {
    float mass_kg;
    float com_m[3];
    float inertia[3][3];
} LinkInertia_t;

/* 连杆外推：{i} 的速度、加速度 -> {i+1} 的速度、加速度
 * joint_rate、joint_accel 为关节 i+1 的转速和角加速度（rad/s、rad/s^2）
 * 速度部分同 Kin_PropagateRevolute，输出可以与输入是同一个对象 */
void Dyn_PropagateAccel(const float T[4][4], const LinkMotion_t *parent, float joint_rate,
                        float joint_accel, LinkMotion_t *child);

/* 力和力矩换到另一个坐标系原点，不含惯性项，输出可以与输入是同一个对象
 * 子 -> 父：f_p = R f_c，n_p = R n_c + p x f_p
 * 父 -> 子：f_c = R^T f_p，n_c = R^T (n_p - p x f_p) */
void Dyn_WrenchToParent(const float T[4][4], const Wrench_t *child, Wrench_t *parent);
void Dyn_WrenchToChild(const float T[4][4], const Wrench_t *parent, Wrench_t *child);

/* 力向内递推：next 是连杆 i 作用在连杆 i+1 上的力（表示在 {i+1}），link、motion 是连杆 i 的惯性参数与运动
 * out 是连杆 i-1 作用在连杆 i 上的力（表示在 {i}），等于 next 换到 {i} 再加上连杆 i 的惯性力和力矩
 * 末端连杆的 next 取末端作用在外界的力，空载时全零，输出可以与 next 是同一个对象 */
void Dyn_PropagateWrench(const float T[4][4], const LinkInertia_t *link,
                         const LinkMotion_t *motion, const Wrench_t *next, Wrench_t *out);

/* 关节链：n 个旋转关节，T[i] 为 {i+1} -> {i}（DH_LinkTransforms 的输出），link[i]、rate[i]、accel[i] 为关节 i+1 的
 * 连杆惯性参数、转速（rad/s）、角加速度（rad/s^2） */
typedef struct {
    const float (*T)[4][4];
    const LinkInertia_t *link;
    const float *rate;
    const float *accel;
    uint8_t n;
} DynChain_t;

#define DYN_GRAVITY_ACC  9.80665f

/* 加速度沿链外推：motion 长度 n + 1，motion[0] = *base，motion[i] 为 {i} 系下 */
void Dyn_ChainAccel(const DynChain_t *chain, const LinkMotion_t *base, LinkMotion_t *motion);

/* 逆动力学：先沿链外推运动，再从末端向基座递推力，得到各关节力矩 torque[n]（N*m）
 * motion 是长度 n + 1 的工作数组，返回时保存各连杆的速度、加速度
 * tip 为末端连杆作用在外界的力（表示在 {n} 系），空载时传 NULL；重力放在 base->accel.linear 里 */
void Dyn_InverseDynamics(const DynChain_t *chain, const LinkMotion_t *base, const Wrench_t *tip,
                         LinkMotion_t *motion, float *torque);

/* 关节力矩：力矩沿关节轴 z 的分量 */
static inline float Dyn_JointTorque(const Wrench_t *wrench)
{
    return wrench->moment[KIN_AXIS_Z];
}

/* 惯量换到父坐标系：绕子系质心的惯量 -> 绕父系原点、表示在父系
 * out = R I R^T + m (|r|^2 E - r r^T)，r = R c + p 为质心在父系下的位置 */
void Dyn_InertiaToFrame(const float T[4][4], const LinkInertia_t *link, float out[3][3]);

#endif

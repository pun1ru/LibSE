#ifndef IK_NUMERIC_H
#define IK_NUMERIC_H

#include <stdbool.h>
#include <stdint.h>

#include "dh.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 七轴臂数值逆解：每个控制周期在当前关节角处线性化，解一步 Δq。移植自 Simulation/ArmSim/Algorithm/ik_numeric，
 * 两边的算法和参数含义保持一致，改动时同步修改并用 ik_numeric_golden 对拍。
 *
 * q 为 7 个关节角（deg 进出，内部 rad），e 为末端 6 维位姿误差，J 为 6×7 几何 Jacobian，W 为关节权重（对角），
 * q_ref 为零空间的参考构型。每步最小化
 *     ‖J·Δq − e‖² + λ²·Δqᵀ W Δq
 * 闭式解（对偶形式，只解 6×6 对称正定方程，Cholesky）：
 *     Δq = W⁻¹Jᵀ(J W⁻¹ Jᵀ + λ²I)⁻¹ e  +  (z − W⁻¹Jᵀ(J W⁻¹ Jᵀ)⁻¹ J z)，z = k·(q_ref − q)
 * 第二项是 z 在 J 零空间上的 W 加权精确投影（无阻尼伪逆，J·dq_null = 0），把关节拉向参考构型而不动末端。
 * 有关节被限位固定后没有冗余度，该项为 0；J W⁻¹ Jᵀ 秩亏时为 0；近奇异时 ‖dq_null‖_W 超过 ‖z‖_W 则按比例缩回。
 * 末端误差量纲统一：e = [Δp(m); L·δφ(rad)]，J 的角速度行同样乘 L，即 1 rad 姿态误差等价于 L 米位置误差。
 * 约束依次施加：关节限位（越界关节钳边界并移出自由集，剩余关节重解，最多 max_limit_passes 轮）、
 * 零空间项只用任务项剩下的步长余量、单周期步长 max_step_deg（超出整体等比缩小）、单周期误差限幅。
 *
 * 内部全部用 double（STM32H7 的 FPU 支持双精度），对外的角度、矩阵、权重用 float，与 dh.h 一致。
 * 单次调用栈占用约 2 KB，不使用动态内存。 */
#define IK_NUMERIC_JOINTS   7U

typedef struct {
    double damping;                 /* λ，在 [m; L·rad] 的量纲下 */
    double orientation_length_m;    /* L：姿态误差折算成位置误差的等效长度 */
    double nullspace_gain;          /* k：每周期向参考构型拉回的比例，0 = 关闭零空间项 */
    double max_step_deg;            /* 每关节每周期最大转角，周期 2 ms 时 0.2° 约 100 °/s */
    double max_pos_err_m;           /* 单步使用的位置误差上限 */
    double max_rot_err_rad;         /* 单步使用的姿态误差上限 */
    double weight_floor;            /* 权重下限，权重 0 会使 W⁻¹ 发散 */
    double converged_pos_m;         /* 位置误差低于此值视为到位 */
    double converged_rot_rad;       /* 姿态误差低于此值视为到位 */
    int max_limit_passes;           /* 限位重解的最大轮数 */
    double max_lead_deg;            /* 指令角领先实测角的上限，由 Arm 层使用：跟不上时不再往前堆 */
} IkNumericParam_t;

typedef struct {
    float dq_deg[IK_NUMERIC_JOINTS];    /* 本周期的关节增量，已满足限位和步长约束 */
    double pos_err_m;                   /* 步前的位置误差范数（未限幅） */
    double rot_err_rad;                 /* 步前的姿态误差角（未限幅） */
    uint8_t limit_mask;                 /* bit i = 第 i 个关节被限位钳住 */
    bool step_limited;                  /* 步长约束起了作用 */
    bool converged;                     /* 步前误差已低于收敛阈值 */
    bool ok;                            /* false：输入非法或线性方程不可解，dq 全 0 */
} IkNumericStep_t;

/* 与 ArmSim 的 NumericIkParam 默认值相同 */
void IkNumeric_DefaultParam(IkNumericParam_t *param);
bool IkNumeric_IsValidParam(const IkNumericParam_t *param);
/* 权重非负有限且总和为正 */
bool IkNumeric_IsValidWeights(const float weight[IK_NUMERIC_JOINTS]);

/* 几何 Jacobian（行 0..2 线速度、3..5 角速度，基座系），第 i 列对应第 i 个关节。
 * frames[k] = T_{0,k+1}（基座到坐标系 k+1），改进 DH 下第 i 个关节绕坐标系 i+1 的 z 轴转，轴线过该坐标系原点 */
void IkNumeric_Jacobian(const float frames[IK_NUMERIC_JOINTS][4][4], double jac[6][IK_NUMERIC_JOINTS]);

/* 位姿误差：e[0..2] = p_d − p_c，e[3..5] = ½ Σ_k r_c,k × r_d,k（小角度下 ≈ 旋转向量，基座系） */
void IkNumeric_PoseError(const float current[4][4], const float desired[4][4], double e[6]);

/* 逐关节累乘的正运动学：frames[k] = T_{0,k+1}，最后一个是末端 */
void IkNumeric_Frames(const DH_Param_t *table, const float q_deg[IK_NUMERIC_JOINTS],
                      float frames[IK_NUMERIC_JOINTS][4][4]);

/* 在 q_deg 处做一步。q_ref_deg 是零空间参考构型；limit_min/max_deg 是多圈限位，与 q_deg 同一圈；
 * weight 是关节权重（越大越不愿动）。输出写进 out，out->ok = false 时 dq 全 0 */
void IkNumeric_Step(const DH_Param_t *table, const float q_deg[IK_NUMERIC_JOINTS], const float target[4][4],
                    const float q_ref_deg[IK_NUMERIC_JOINTS], const float weight[IK_NUMERIC_JOINTS],
                    const float limit_min_deg[IK_NUMERIC_JOINTS], const float limit_max_deg[IK_NUMERIC_JOINTS],
                    const IkNumericParam_t *param, IkNumericStep_t *out);

#ifdef __cplusplus
}
#endif

#endif

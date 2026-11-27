#ifndef ARM_IK_H
#define ARM_IK_H

#include "arm.h"

/* 逆解的 Arm 层接口，算法本体在 ik_numeric.h / ik_analytic.h。在 ARM_HOST / ARM_IK 下有效：
 * NUMERIC 给出末端位姿后每个控制周期推进一步；ANALYTIC 在设置目标时一次求解全部关节。
 * 没到位却几乎不动达到这个周期数时置 ik.stalled（2 ms 周期下 0.5 s） */
#define ARM_IK_STALL_CYCLES     250U
#define ARM_IK_STALL_STEP_DEG   1e-3f

typedef enum {
    ARM_IK_OK = 0,
    ARM_IK_WRONG_STATE,     /* 不在 ARM_HOST / ARM_IK */
    ARM_IK_BAD_PARAM,       /* 位姿含非有限值、权重非法或参数表没填 */
    ARM_IK_BAD_BRANCH,
    ARM_IK_OUT_OF_REACH,
    ARM_IK_REF_DEGENERATE,
    ARM_IK_OUT_OF_LIMIT
} ArmIkResult_t;

/* Defaults to ARM_IK_TYPE_NUMERIC in Arm_Init. Changing type stops any active IK. */
bool Arm_SetIkType(Arm_t *arm, ArmIkType_t type);

/* Dispatch by arm->ik_type. For the numeric solver branch and psi are ignored.
 * For the analytic solver branch must be valid and psi_deg is used directly. The
 * concrete setter below also selects its own type, so direct calls remain coherent. */
ArmIkResult_t Arm_SetIkTarget(Arm_t *arm, const float target[4][4],
                              const IkAnalyticBranch_t *branch, float psi_deg,
                              const float *weight);

/* 设定末端目标位姿（基座系，4x4 齐次变换）。weight 传 NULL 用 ArmParam_t.ik.weight。
 * 首次进入时关节目标对齐当前角，零空间参考构型取当前目标；已在数值逆解里更新目标时参考构型保持不变。
 * 成功后 7 个关节全部进入闭环。 */
ArmIkResult_t Arm_SetNumericIkTarget(Arm_t *arm, const float target[4][4], const float *weight);
ArmIkResult_t Arm_SetAnalyticIkTarget(Arm_t *arm, const float target[4][4],
                                      const IkAnalyticBranch_t *branch,
                                      float psi_deg);
/* 退出逆解，关节目标停在当前值，不改闭环掩码 */
void Arm_StopIk(Arm_t *arm);
/* Dispatch: numeric advances one differential step, analytic is solved once on target entry. */
void Arm_StepIk(Arm_t *arm);
/* 每周期在 Arm_UpdateTarget 里调用：以上一周期的关节目标为线性化点走一步（不用实测角：PID 的积分项会补偿
 * 重力和摩擦造成的静差，以实测角为起点反而会让指令只领先实测一小步而推不动）。
 * 指令领先实测角超过 max_lead_deg 时钳回，机构跟不上时指令不会一直往前堆。求解失败时保持上一周期目标。 */
void Arm_StepNumericIk(Arm_t *arm);

#endif

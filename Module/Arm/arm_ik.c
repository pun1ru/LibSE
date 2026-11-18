#include "arm_ik.h"

#include <math.h>
#include <string.h>

#include "arm_jointctrl.h"

#define ARM_IK_ALL_JOINTS_MASK  ((uint8_t)((1U << ARM_JOINT_COUNT) - 1U))

static float Arm_IkClamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void Arm_StopIk(Arm_t *arm)
{
    if (arm != NULL)
    {
        arm->ik.active = 0U;
    }
}

ArmIkResult_t Arm_SetNumericIkTarget(Arm_t *arm, const float target[4][4], const float *weight)
{
    const ArmIkParam_t *param;
    const float *use_weight;
    bool keep_reference;

    if (arm == NULL || target == NULL)
    {
        return ARM_IK_BAD_PARAM;
    }
    if (arm->state != ARM_HOST)
    {
        return ARM_IK_WRONG_STATE;
    }
    param = &arm->config.param.ik;
    use_weight = weight != NULL ? weight : param->weight;
    if (!IkNumeric_IsValidParam(&param->numeric) || !IkNumeric_IsValidWeights(use_weight))
    {
        return ARM_IK_BAD_PARAM;
    }
    for (unsigned int r = 0U; r < 3U; ++r)
    {
        for (unsigned int c = 0U; c < 4U; ++c)
        {
            if (!isfinite(target[r][c]))
            {
                return ARM_IK_BAD_PARAM;
            }
        }
    }

    /* 目标可能先于第一个 HOST 控制周期到达：先锁存本次进入的限位并对齐，再设目标 */
    if (arm->host_align_pending != 0U)
    {
        Arm_AlignHostTarget(arm);
        arm->host_align_pending = 0U;
        arm->ctrl_align_pending = 0U;
    }

    keep_reference = arm->ik.active != 0U;
    if (!keep_reference)
    {
        for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
        {
            arm->ik.reference_deg[i] = arm->joint_ctrl[i].target_deg;
        }
        arm->ik.stall_cycles = 0U;
        arm->ik.stalled = 0U;
    }
    for (unsigned int r = 0U; r < 4U; ++r)
    {
        for (unsigned int c = 0U; c < 4U; ++c)
        {
            arm->ik.target[r][c] = target[r][c];
        }
    }
    memcpy(arm->ik.weight, use_weight, sizeof(arm->ik.weight));

    /* 新进入闭环的关节 PID 从零开始，已在闭环的保持积分 */
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        if ((arm->host_closed_loop_mask & (uint8_t)(1U << i)) == 0U)
        {
            PIDReset(&arm->joint_ctrl[i].pid);
        }
        arm->host_target_deg[i] = arm->joint_ctrl[i].target_deg;
    }
    arm->host_target_valid_mask = ARM_IK_ALL_JOINTS_MASK;
    arm->host_closed_loop_mask = ARM_IK_ALL_JOINTS_MASK;
    arm->ik.active = 1U;
    return ARM_IK_OK;
}

void Arm_StepNumericIk(Arm_t *arm)
{
    const IkNumericParam_t *param = &arm->config.param.ik.numeric;
    ArmIkState_t *ik = &arm->ik;
    IkNumericStep_t step;
    float q[ARM_JOINT_COUNT];
    float limit_min[ARM_JOINT_COUNT];
    float limit_max[ARM_JOINT_COUNT];
    float peak_deg = 0.0f;

    if (ik->active == 0U || arm->state != ARM_HOST)
    {
        return;
    }
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        q[i] = arm->joint_ctrl[i].target_deg;
        limit_min[i] = arm->joint_ctrl[i].limit_min_deg;
        limit_max[i] = arm->joint_ctrl[i].limit_max_deg;
    }

    IkNumeric_Step(arm->config.param.dh, q, (const float (*)[4])ik->target, ik->reference_deg, ik->weight,
                   limit_min, limit_max, param, &step);
    if (!step.ok)
    {
        return;   /* 保持上一周期的目标 */
    }

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        const float lead = (float)param->max_lead_deg;
        const float observed = arm->observer.joint_deg[i];
        float next = Arm_IkClamp(q[i] + step.dq_deg[i], observed - lead, observed + lead);

        next = Arm_IkClamp(next, limit_min[i], limit_max[i]);
        peak_deg = fmaxf(peak_deg, fabsf(next - q[i]));
        arm->joint_ctrl[i].target_deg = next;
        arm->host_target_deg[i] = next;
    }

    ik->pos_err_m = (float)step.pos_err_m;
    ik->rot_err_deg = (float)(step.rot_err_rad * (180.0 / 3.14159265358979323846));
    ik->limit_mask = step.limit_mask;
    ik->step_limited = step.step_limited ? 1U : 0U;
    ik->converged = step.converged ? 1U : 0U;

    /* 没到位却几乎不动：目标够不到，或被限位 / 奇异挡住 */
    if (!step.converged && peak_deg < ARM_IK_STALL_STEP_DEG)
    {
        if (ik->stall_cycles < ARM_IK_STALL_CYCLES)
        {
            ++ik->stall_cycles;
        }
    }
    else
    {
        ik->stall_cycles = 0U;
    }
    ik->stalled = ik->stall_cycles >= ARM_IK_STALL_CYCLES ? 1U : 0U;
}

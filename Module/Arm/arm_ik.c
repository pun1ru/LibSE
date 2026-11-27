#include "arm_ik.h"

#include <math.h>
#include <string.h>

#include "algorism.h"
#include "arm_jointctrl.h"

#define ARM_IK_ALL_JOINTS_MASK  ((uint8_t)((1U << ARM_JOINT_COUNT) - 1U))
#define ARM_IK_RAD_TO_DEG       (180.0f / 3.14159265358979323846f)

static float Arm_WrapPiDeg(float angle_deg)
{
    float wrapped = fmodf(angle_deg + 180.0f, 360.0f);

    if (wrapped < 0.0f) {
        wrapped += 360.0f;
    }
    return wrapped - 180.0f;
}

static bool Arm_MapAnalyticTargets(const Arm_t *arm,
                                   const IkAnalyticResult_t *solution,
                                   const float reference_deg[ARM_JOINT_COUNT],
                                   float output_deg[ARM_JOINT_COUNT])
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
        const float multi_deg = arm->observer.joint_deg[i];
        const float abs_deg = arm->observer.joint_abs_deg[i];
        const float base_deg =
            multi_deg + Arm_WrapPiDeg(solution->q_deg[i] - abs_deg);
        const float limit_min_deg =
            multi_deg + (arm->config.param.limit[i].min_deg - abs_deg);
        const float limit_max_deg =
            multi_deg + (arm->config.param.limit[i].max_deg - abs_deg);
        float best_deg = 0.0f;
        float best_error = 0.0f;
        bool found = false;

        if (!(limit_min_deg < limit_max_deg)) {
            return false;
        }
        for (int8_t turns = -2; turns <= 2; ++turns) {
            const float candidate =
                base_deg + (float)turns * 360.0f;
            float error;

            if (candidate < limit_min_deg || candidate > limit_max_deg) {
                continue;
            }
            error = fabsf(candidate - reference_deg[i]);
            if (!found || error < best_error) {
                found = true;
                best_deg = candidate;
                best_error = error;
            }
        }
        if (!found) {
            return false;
        }
        output_deg[i] = best_deg;
    }
    return true;
}

static void Arm_RecordAnalyticPoseError(Arm_t *arm,
                                        const float target[4][4])
{
    float current[4][4];
    float q_deg[ARM_JOINT_COUNT];
    float position_sq = 0.0f;
    float trace = 0.0f;

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
        q_deg[i] = arm->joint_ctrl[i].target_deg;
    }
    DH_ForwardKinematics(arm->config.param.dh, q_deg, ARM_JOINT_COUNT,
                         current);
    for (uint8_t i = 0U; i < 3U; ++i) {
        const float delta = current[i][3] - target[i][3];

        position_sq += delta * delta;
    }
    for (uint8_t row = 0U; row < 3U; ++row) {
        for (uint8_t column = 0U; column < 3U; ++column) {
            trace += current[row][column] * target[row][column];
        }
    }
    arm->ik.pos_err_m = sqrtf(position_sq);
    {
        float cosine = 0.5f * (trace - 1.0f);

        cosine = cosine > 1.0f ? 1.0f : (cosine < -1.0f ? -1.0f : cosine);
        arm->ik.rot_err_deg = acosf(cosine) * ARM_IK_RAD_TO_DEG;
    }
}

bool Arm_SetIkType(Arm_t *arm, ArmIkType_t type)
{
    if (arm == NULL ||
        (type != ARM_IK_TYPE_NUMERIC && type != ARM_IK_TYPE_ANALYTIC)) {
        return false;
    }
    if (arm->ik_type != type) {
        Arm_StopIk(arm);
        arm->ik_type = type;
    }
    return true;
}

void Arm_StopIk(Arm_t *arm)
{
    if (arm != NULL) {
        arm->ik.active = 0U;
        arm->ik.analytic_valid = 0U;
    }
}

ArmIkResult_t Arm_SetIkTarget(Arm_t *arm, const float target[4][4],
                              const IkAnalyticBranch_t *branch, float psi_deg,
                              const float *weight)
{
    if (arm == NULL || target == NULL) {
        return ARM_IK_BAD_PARAM;
    }
    if (arm->ik_type == ARM_IK_TYPE_NUMERIC) {
        return Arm_SetNumericIkTarget(arm, target, weight);
    }
    return Arm_SetAnalyticIkTarget(arm, target, branch, psi_deg);
}

ArmIkResult_t Arm_SetNumericIkTarget(Arm_t *arm, const float target[4][4], const float *weight)
{
    const ArmIkParam_t *param;
    const float *use_weight;
    bool keep_reference;

    if (arm->state != ARM_HOST && arm->state != ARM_IK)
    {
        return ARM_IK_WRONG_STATE;
    }
    (void)Arm_SetIkType(arm, ARM_IK_TYPE_NUMERIC);
    param = &arm->config.param.ik;
    use_weight = weight != NULL ? weight : param->weight;
    if (!IkNumeric_IsValidParam(&param->numeric) || !IkNumeric_IsValidWeights(use_weight))
    {
        return ARM_IK_BAD_PARAM;
    }
    /* 目标可能先于第一个 HOST 控制周期到达：先锁存本次进入的限位并对齐，再设目标 */
    if (arm->host.host_align_pending != 0U)
    {
        Arm_AlignHostTarget(arm);
        arm->host.host_align_pending = 0U;
        arm->host.ctrl_align_pending = 0U;
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
        if ((arm->host.host_closed_loop_mask & (uint8_t)(1U << i)) == 0U)
        {
            PIDReset(&arm->joint_ctrl[i].pid);
        }
        arm->host.host_target_deg[i] = arm->joint_ctrl[i].target_deg;
    }
    arm->host.host_target_valid_mask = ARM_IK_ALL_JOINTS_MASK;
    arm->host.host_closed_loop_mask = ARM_IK_ALL_JOINTS_MASK;
    arm->ik.active = 1U;
    arm->ik.analytic_valid = 0U;
    return ARM_IK_OK;
}

ArmIkResult_t Arm_SetAnalyticIkTarget(Arm_t *arm, const float target[4][4],
                                      const IkAnalyticBranch_t *branch,
                                      float psi_deg)
{
    IkAnalyticSrsParam_t srs;
    IkAnalyticResult_t solution;
    float previous_target_deg[ARM_JOINT_COUNT];
    float next_target_deg[ARM_JOINT_COUNT];

    if (arm == NULL || target == NULL) {
        return ARM_IK_BAD_PARAM;
    }
    if (arm->state != ARM_HOST && arm->state != ARM_IK) {
        return ARM_IK_WRONG_STATE;
    }
    if (branch == NULL || !branch->valid) {
        return ARM_IK_BAD_BRANCH;
    }
    if (!isfinite(psi_deg) ||
        !IkAnalytic_MakeSrsParam(arm->config.param.dh, &srs)) {
        return ARM_IK_BAD_PARAM;
    }
    (void)Arm_SetIkType(arm, ARM_IK_TYPE_ANALYTIC);

    if (arm->host.host_align_pending != 0U) {
        Arm_AlignHostTarget(arm);
        arm->host.host_align_pending = 0U;
        arm->host.ctrl_align_pending = 0U;
    }
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
        previous_target_deg[i] = arm->joint_ctrl[i].target_deg;
    }

    IkAnalytic_Solve(&srs, target, branch, psi_deg, &solution);
    if (solution.status == IK_ANALYTIC_OUT_OF_REACH) {
        return ARM_IK_OUT_OF_REACH;
    }
    if (solution.status == IK_ANALYTIC_REF_DEGENERATE) {
        return ARM_IK_REF_DEGENERATE;
    }
    if (solution.status != IK_ANALYTIC_OK) {
        return ARM_IK_BAD_PARAM;
    }
    if (!Arm_MapAnalyticTargets(arm, &solution, previous_target_deg,
                                next_target_deg)) {
        return ARM_IK_OUT_OF_LIMIT;
    }

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
        const uint8_t bit = (uint8_t)(1U << i);

        if ((arm->host.host_closed_loop_mask & bit) == 0U) {
            PIDReset(&arm->joint_ctrl[i].pid);
        }
        arm->joint_ctrl[i].target_deg = next_target_deg[i];
        arm->host.host_target_deg[i] = next_target_deg[i];
        arm->ik.reference_deg[i] = previous_target_deg[i];
    }
    for (uint8_t row = 0U; row < 4U; ++row) {
        for (uint8_t column = 0U; column < 4U; ++column) {
            arm->ik.target[row][column] = target[row][column];
        }
    }
    arm->host.host_target_valid_mask = ARM_IK_ALL_JOINTS_MASK;
    arm->host.host_closed_loop_mask = ARM_IK_ALL_JOINTS_MASK;
    arm->ik.active = 1U;
    arm->ik.limit_mask = 0U;
    arm->ik.step_limited = 0U;
    arm->ik.converged = 1U;
    arm->ik.stalled = 0U;
    arm->ik.stall_cycles = 0U;
    arm->ik.analytic_branch = *branch;
    arm->ik.analytic_psi_deg = psi_deg;
    arm->ik.analytic_valid = 1U;
    arm->ik.analytic_shoulder_singular = solution.shoulder_singular;
    arm->ik.analytic_wrist_singular = solution.wrist_singular;
    Arm_RecordAnalyticPoseError(arm, target);
    return ARM_IK_OK;
}

void Arm_StepIk(Arm_t *arm)
{
    if (arm == NULL) {
        return;
    }
    switch (arm->ik_type) {
    case ARM_IK_TYPE_NUMERIC:
        Arm_StepNumericIk(arm);
        break;
    case ARM_IK_TYPE_ANALYTIC:
        /* Analytic targets are solved once when they are set. */
        break;
    default:
        break;
    }
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

    if (ik->active == 0U ||
        (arm->state != ARM_HOST && arm->state != ARM_IK))
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
        float next = DoubleEdgeLimiter(q[i] + step.dq_deg[i], observed - lead, observed + lead);

        next = DoubleEdgeLimiter(next, limit_min[i], limit_max[i]);
        peak_deg = fmaxf(peak_deg, fabsf(next - q[i]));
        arm->joint_ctrl[i].target_deg = next;
        arm->host.host_target_deg[i] = next;
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

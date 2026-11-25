#include "ik_analytic.h"

#include <math.h>
#include <string.h>

#define IK_ANALYTIC_PI          3.14159265358979323846
#define IK_ANALYTIC_TWO_PI      (2.0 * IK_ANALYTIC_PI)
#define IK_ANALYTIC_DEG_TO_RAD  (IK_ANALYTIC_PI / 180.0)
#define IK_ANALYTIC_RAD_TO_DEG  (180.0 / IK_ANALYTIC_PI)
#define IK_ANALYTIC_SINGULAR_EPS 1e-6
#define IK_ANALYTIC_REACH_EPS    1e-5
#define IK_ANALYTIC_MIN_LEN      1e-9
#define IK_ANALYTIC_REF_EPS      1e-9
#define IK_ANALYTIC_ARM_MIN_SIN  0.02

typedef struct {
    double value[3];
} AnalyticVec3_t;

typedef struct {
    double value[3][3];
} AnalyticMat3_t;

static AnalyticVec3_t VecAdd(AnalyticVec3_t a, AnalyticVec3_t b)
{
    AnalyticVec3_t out = {{a.value[0] + b.value[0], a.value[1] + b.value[1],
                           a.value[2] + b.value[2]}};
    return out;
}

static AnalyticVec3_t VecSub(AnalyticVec3_t a, AnalyticVec3_t b)
{
    AnalyticVec3_t out = {{a.value[0] - b.value[0], a.value[1] - b.value[1],
                           a.value[2] - b.value[2]}};
    return out;
}

static AnalyticVec3_t VecScale(AnalyticVec3_t a, double scale)
{
    AnalyticVec3_t out = {{a.value[0] * scale, a.value[1] * scale,
                           a.value[2] * scale}};
    return out;
}

static double VecDot(AnalyticVec3_t a, AnalyticVec3_t b)
{
    return a.value[0] * b.value[0] + a.value[1] * b.value[1] +
           a.value[2] * b.value[2];
}

static double VecNorm(AnalyticVec3_t a)
{
    return sqrt(VecDot(a, a));
}

static AnalyticVec3_t VecCross(AnalyticVec3_t a, AnalyticVec3_t b)
{
    AnalyticVec3_t out = {{
        a.value[1] * b.value[2] - a.value[2] * b.value[1],
        a.value[2] * b.value[0] - a.value[0] * b.value[2],
        a.value[0] * b.value[1] - a.value[1] * b.value[0]
    }};
    return out;
}

static AnalyticVec3_t MatCol(AnalyticMat3_t matrix, uint8_t column)
{
    AnalyticVec3_t out = {{
        matrix.value[0][column],
        matrix.value[1][column],
        matrix.value[2][column]
    }};
    return out;
}

static AnalyticMat3_t MatFromColumns(AnalyticVec3_t x, AnalyticVec3_t y,
                                     AnalyticVec3_t z)
{
    AnalyticMat3_t out = {{
        {x.value[0], y.value[0], z.value[0]},
        {x.value[1], y.value[1], z.value[1]},
        {x.value[2], y.value[2], z.value[2]}
    }};
    return out;
}

static AnalyticMat3_t MatMul(AnalyticMat3_t a, AnalyticMat3_t b)
{
    AnalyticMat3_t out = {{{0.0}}};

    for (uint8_t row = 0U; row < 3U; ++row) {
        for (uint8_t column = 0U; column < 3U; ++column) {
            out.value[row][column] =
                a.value[row][0] * b.value[0][column] +
                a.value[row][1] * b.value[1][column] +
                a.value[row][2] * b.value[2][column];
        }
    }
    return out;
}

static AnalyticMat3_t MatTranspose(AnalyticMat3_t a)
{
    AnalyticMat3_t out = {{{0.0}}};

    for (uint8_t row = 0U; row < 3U; ++row) {
        for (uint8_t column = 0U; column < 3U; ++column) {
            out.value[row][column] = a.value[column][row];
        }
    }
    return out;
}

static AnalyticMat3_t MatRotZ(double angle)
{
    const double c = cos(angle);
    const double s = sin(angle);
    AnalyticMat3_t out = {{{
        c, -s, 0.0
    }, {
        s, c, 0.0
    }, {
        0.0, 0.0, 1.0
    }}};
    return out;
}

static AnalyticMat3_t MatRodrigues(AnalyticVec3_t axis, double angle)
{
    const double c = cos(angle);
    const double s = sin(angle);
    const double v = 1.0 - c;
    const double x = axis.value[0];
    const double y = axis.value[1];
    const double z = axis.value[2];
    AnalyticMat3_t out = {{{
        c + x * x * v, x * y * v - z * s, x * z * v + y * s
    }, {
        y * x * v + z * s, c + y * y * v, y * z * v - x * s
    }, {
        z * x * v - y * s, z * y * v + x * s, c + z * z * v
    }}};
    return out;
}

static AnalyticVec3_t MatMulVec(AnalyticMat3_t matrix, AnalyticVec3_t vector)
{
    AnalyticVec3_t out = {{
        matrix.value[0][0] * vector.value[0] +
        matrix.value[0][1] * vector.value[1] +
        matrix.value[0][2] * vector.value[2],
        matrix.value[1][0] * vector.value[0] +
        matrix.value[1][1] * vector.value[1] +
        matrix.value[1][2] * vector.value[2],
        matrix.value[2][0] * vector.value[0] +
        matrix.value[2][1] * vector.value[1] +
        matrix.value[2][2] * vector.value[2]
    }};
    return out;
}

static AnalyticVec3_t MatRotateVector(AnalyticVec3_t axis, double angle,
                                      AnalyticVec3_t vector)
{
    return MatMulVec(MatRodrigues(axis, angle), vector);
}

static AnalyticMat3_t MatRotationOf(const float transform[4][4])
{
    AnalyticMat3_t out = {{{0.0}}};

    for (uint8_t row = 0U; row < 3U; ++row) {
        for (uint8_t column = 0U; column < 3U; ++column) {
            out.value[row][column] = (double)transform[row][column];
        }
    }
    return out;
}

static AnalyticVec3_t MatOriginOf(const float transform[4][4])
{
    AnalyticVec3_t out = {{
        (double)transform[0][3],
        (double)transform[1][3],
        (double)transform[2][3]
    }};
    return out;
}

static double WrapRad(double angle)
{
    angle = fmod(angle + IK_ANALYTIC_PI, IK_ANALYTIC_TWO_PI);
    if (angle < 0.0) {
        angle += IK_ANALYTIC_TWO_PI;
    }
    return angle - IK_ANALYTIC_PI;
}

static bool NearDeg(float value, float target)
{
    return fabs((double)value - (double)target) < 1e-3;
}

static bool IsZero(float value)
{
    return fabs((double)value) < 1e-6;
}

static bool IsRightAngle(float value)
{
    return fabs(fabs((double)value) - 90.0) < 1e-3;
}

static bool IsSrsStructure(const DH_Param_t table[IK_ANALYTIC_JOINTS])
{
    const bool shoulder = IsZero(table[1].a_m) && IsZero(table[1].d_m) &&
                          IsZero(table[2].a_m) &&
                          IsRightAngle(table[1].alpha_deg) &&
                          IsRightAngle(table[2].alpha_deg);
    const bool elbow = IsZero(table[3].a_m) && IsZero(table[4].a_m) &&
                       IsRightAngle(table[3].alpha_deg) &&
                       IsRightAngle(table[4].alpha_deg);
    const bool wrist = IsZero(table[5].a_m) && IsZero(table[5].d_m) &&
                       IsZero(table[6].a_m) &&
                       IsRightAngle(table[5].alpha_deg) &&
                       IsRightAngle(table[6].alpha_deg);

    return shoulder && elbow && wrist;
}

static bool DecomposeZxzxz(AnalyticMat3_t rotation, int8_t lambda,
                           double *angle_a, double *angle_b, double *angle_c)
{
    const double cb = -rotation.value[2][2];
    const double sb = hypot(rotation.value[0][2], rotation.value[1][2]);

    if (sb < IK_ANALYTIC_SINGULAR_EPS) {
        *angle_a = 0.0;
        *angle_b = cb > 0.0 ? 0.0 : IK_ANALYTIC_PI;
        *angle_c = atan2(-rotation.value[1][0], cb * rotation.value[0][0]);
        return true;
    }

    {
        const double l = lambda < 0 ? -1.0 : 1.0;

        *angle_b = atan2(l * sb, cb);
        *angle_a = atan2(l * rotation.value[1][2],
                         l * rotation.value[0][2]);
        *angle_c = atan2(-l * rotation.value[2][1],
                         l * rotation.value[2][0]);
    }
    return false;
}

static bool SolveElbowAngle(const IkAnalyticSrsParam_t *param,
                            double shoulder_wrist_dist_m,
                            int8_t elbow_branch, double *theta4_rad)
{
    const double upper = param->upper_arm_m;
    const double offset = param->elbow_offset_m;
    const double forearm = param->forearm_m;
    const double cosine =
        (upper * upper + forearm * forearm + offset * offset -
         shoulder_wrist_dist_m * shoulder_wrist_dist_m) /
        (2.0 * upper * forearm);

    if (!(fabs(cosine) <= 1.0 + IK_ANALYTIC_REACH_EPS)) {
        return false;
    }
    {
        const double clipped = cosine > 1.0 ? 1.0 :
                               (cosine < -1.0 ? -1.0 : cosine);
        const double angle = acos(clipped);

        *theta4_rad = elbow_branch < 0 ? -angle : angle;
    }
    return true;
}

static bool ReferenceElbowAxis(const IkAnalyticSrsParam_t *param,
                               AnalyticVec3_t shoulder_wrist,
                               AnalyticVec3_t *elbow_axis)
{
    const double length = VecNorm(shoulder_wrist);
    AnalyticVec3_t unit;
    double sin_gamma;
    double cos_phi;
    double sin_phi;
    AnalyticVec3_t horizontal;
    AnalyticVec3_t normal;

    if (length < IK_ANALYTIC_MIN_LEN) {
        return false;
    }
    unit = VecScale(shoulder_wrist, 1.0 / length);
    sin_gamma = hypot(unit.value[0], unit.value[1]);
    if (sin_gamma < IK_ANALYTIC_ARM_MIN_SIN) {
        return false;
    }
    cos_phi = -(param->elbow_offset_m / length) / sin_gamma;
    if (fabs(cos_phi) > 1.0 - IK_ANALYTIC_REF_EPS) {
        return false;
    }
    sin_phi = sqrt(1.0 - cos_phi * cos_phi);
    horizontal = (AnalyticVec3_t){{unit.value[0] / sin_gamma,
                                   unit.value[1] / sin_gamma, 0.0}};
    normal = (AnalyticVec3_t){{-horizontal.value[1],
                               horizontal.value[0], 0.0}};
    *elbow_axis = VecAdd(VecScale(horizontal, cos_phi),
                         VecScale(normal, sin_phi));
    return true;
}

static bool ReferenceR03(const IkAnalyticSrsParam_t *param,
                         AnalyticVec3_t shoulder_wrist,
                         AnalyticVec3_t elbow_axis, double theta4,
                         AnalyticMat3_t *rotation_03)
{
    const AnalyticVec3_t offset_point =
        VecAdd(shoulder_wrist, VecScale(elbow_axis, param->elbow_offset_m));
    const double length = VecNorm(offset_point);
    AnalyticVec3_t unit;
    double delta;
    AnalyticVec3_t axis_z;
    AnalyticVec3_t axis_x;

    if (length < IK_ANALYTIC_MIN_LEN) {
        return false;
    }
    unit = VecScale(offset_point, 1.0 / length);
    delta = atan2(param->forearm_m * sin(theta4),
                  param->upper_arm_m - param->forearm_m * cos(theta4));
    axis_z = MatRotateVector(elbow_axis, -delta, unit);
    axis_x = VecCross(elbow_axis, axis_z);
    *rotation_03 = MatFromColumns(axis_x, elbow_axis, axis_z);
    return true;
}

bool IkAnalytic_MakeSrsParam(const DH_Param_t table[IK_ANALYTIC_JOINTS],
                             IkAnalyticSrsParam_t *out)
{
    if (table == NULL || out == NULL || !IsSrsStructure(table) ||
        !NearDeg(table[0].alpha_deg, 0.0f) ||
        !(table[2].d_m > 0.0f) || !(table[4].d_m > 0.0f)) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->shoulder_m[0] = (double)table[0].a_m;
    out->shoulder_m[1] = 0.0;
    out->shoulder_m[2] = (double)table[0].d_m;
    out->upper_arm_m = (double)table[2].d_m;
    out->elbow_offset_m = (double)table[3].d_m;
    out->forearm_m = (double)table[4].d_m;
    out->tool_m = (double)table[6].d_m;
    for (uint8_t i = 0U; i < IK_ANALYTIC_JOINTS; ++i) {
        out->theta_offset_rad[i] =
            (double)table[i].theta_offset_deg * IK_ANALYTIC_DEG_TO_RAD;
    }
    return true;
}

bool IkAnalytic_ComputeBranch(const DH_Param_t table[IK_ANALYTIC_JOINTS],
                              const float q_deg[IK_ANALYTIC_JOINTS],
                              IkAnalyticBranch_t *out)
{
    static const uint8_t indices[3] = {1U, 3U, 5U};

    if (table == NULL || q_deg == NULL || out == NULL) {
        return false;
    }
    out->valid = true;
    for (uint8_t i = 0U; i < 3U; ++i) {
        const uint8_t index = indices[i];
        const double theta =
            ((double)q_deg[index] + (double)table[index].theta_offset_deg) *
            IK_ANALYTIC_DEG_TO_RAD;
        const int8_t side = sin(theta) >= 0.0 ? 1 : -1;

        if (i == 0U) {
            out->shoulder = side;
        } else if (i == 1U) {
            out->elbow = side;
        } else {
            out->wrist = side;
        }
    }
    return true;
}

uint8_t IkAnalytic_BranchIndex(const IkAnalyticBranch_t *branch)
{
    if (branch == NULL || !branch->valid) {
        return IK_ANALYTIC_BRANCH_COUNT;
    }
    return (uint8_t)((branch->shoulder < 0 ? 1U : 0U) |
                     (branch->elbow < 0 ? 2U : 0U) |
                     (branch->wrist < 0 ? 4U : 0U));
}

void IkAnalytic_BranchFromIndex(uint8_t index, IkAnalyticBranch_t *branch)
{
    if (branch == NULL) {
        return;
    }
    branch->valid = index < IK_ANALYTIC_BRANCH_COUNT ? true : false;
    branch->shoulder = (index & 1U) != 0U ? -1 : 1;
    branch->elbow = (index & 2U) != 0U ? -1 : 1;
    branch->wrist = (index & 4U) != 0U ? -1 : 1;
}

void IkAnalytic_Solve(const IkAnalyticSrsParam_t *param,
                      const float target[4][4],
                      const IkAnalyticBranch_t *branch,
                      float psi_deg,
                      IkAnalyticResult_t *out)
{
    AnalyticMat3_t rotation_07;
    AnalyticVec3_t target_position;
    AnalyticVec3_t wrist;
    AnalyticVec3_t shoulder_wrist;
    AnalyticVec3_t shoulder;
    double length;
    double theta4;
    AnalyticVec3_t elbow_axis_ref;
    AnalyticMat3_t rotation_03_ref;
    AnalyticVec3_t sw_unit;
    AnalyticMat3_t rotation_03;
    AnalyticMat3_t rotation_04;
    AnalyticMat3_t matrix;
    double theta1;
    double theta2;
    double theta3;
    double theta5;
    double theta6;
    double theta7;

    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->status = IK_ANALYTIC_BAD_PARAM;
    if (param == NULL || target == NULL || branch == NULL || !branch->valid) {
        return;
    }

    rotation_07 = MatRotationOf(target);
    target_position = MatOriginOf(target);
    wrist = VecSub(target_position,
                   VecScale(MatCol(rotation_07, 2U), param->tool_m));
    shoulder = (AnalyticVec3_t){{param->shoulder_m[0],
                                 param->shoulder_m[1],
                                 param->shoulder_m[2]}};
    shoulder_wrist = VecSub(wrist, shoulder);
    length = VecNorm(shoulder_wrist);

    if (!SolveElbowAngle(param, length, branch->elbow, &theta4)) {
        out->status = IK_ANALYTIC_OUT_OF_REACH;
        return;
    }
    if (!ReferenceElbowAxis(param, shoulder_wrist, &elbow_axis_ref) ||
        !ReferenceR03(param, shoulder_wrist, elbow_axis_ref, theta4,
                      &rotation_03_ref)) {
        out->status = IK_ANALYTIC_REF_DEGENERATE;
        return;
    }

    sw_unit = VecScale(shoulder_wrist, 1.0 / length);
    rotation_03 = MatMul(
        MatRodrigues(sw_unit,
                     (double)psi_deg * IK_ANALYTIC_DEG_TO_RAD),
        rotation_03_ref);

    out->shoulder_singular =
        DecomposeZxzxz(rotation_03, branch->shoulder,
                       &theta1, &theta2, &theta3) ? 1U : 0U;

    {
        static const AnalyticMat3_t rx90 = {{{
            1.0, 0.0, 0.0
        }, {
            0.0, 0.0, -1.0
        }, {
            0.0, 1.0, 0.0
        }}};
        rotation_04 = MatMul(MatMul(rotation_03, rx90), MatRotZ(theta4));
        matrix = MatMul(MatMul(MatTranspose(rx90),
                               MatTranspose(rotation_04)),
                        rotation_07);
        out->wrist_singular =
            DecomposeZxzxz(matrix, branch->wrist,
                           &theta5, &theta6, &theta7) ? 1U : 0U;
    }

    {
        const double theta[IK_ANALYTIC_JOINTS] = {
            theta1, theta2, theta3, theta4, theta5, theta6, theta7
        };

        for (uint8_t i = 0U; i < IK_ANALYTIC_JOINTS; ++i) {
            out->q_deg[i] = (float)(
                WrapRad(theta[i] - param->theta_offset_rad[i]) *
                IK_ANALYTIC_RAD_TO_DEG);
        }
    }
    out->status = IK_ANALYTIC_OK;
}

void IkAnalytic_SolveAllBranches(const IkAnalyticSrsParam_t *param,
                                 const float target[4][4],
                                 float psi_deg,
                                 IkAnalyticResult_t out[IK_ANALYTIC_BRANCH_COUNT])
{
    if (out == NULL) {
        return;
    }
    for (uint8_t i = 0U; i < IK_ANALYTIC_BRANCH_COUNT; ++i) {
        IkAnalyticBranch_t branch;

        IkAnalytic_BranchFromIndex(i, &branch);
        IkAnalytic_Solve(param, target, &branch, psi_deg, &out[i]);
    }
}

bool IkAnalytic_ComputeArmAngle(const DH_Param_t table[IK_ANALYTIC_JOINTS],
                                const IkAnalyticSrsParam_t *param,
                                const float q_deg[IK_ANALYTIC_JOINTS],
                                float *psi_deg)
{
    float frames[IK_ANALYTIC_JOINTS][4][4];
    float current[4][4];
    AnalyticVec3_t shoulder;
    AnalyticVec3_t wrist;
    AnalyticVec3_t shoulder_wrist;
    double length;
    AnalyticVec3_t elbow_axis_ref;
    AnalyticVec3_t unit;
    AnalyticMat3_t rotation_03;
    AnalyticVec3_t elbow_axis;
    double dot_ref;
    double dot_actual;
    double psi;

    if (table == NULL || param == NULL || q_deg == NULL || psi_deg == NULL) {
        return false;
    }
    memset(current, 0, sizeof(current));
    current[0][0] = 1.0f;
    current[1][1] = 1.0f;
    current[2][2] = 1.0f;
    current[3][3] = 1.0f;
    for (uint8_t i = 0U; i < IK_ANALYTIC_JOINTS; ++i) {
        float link[4][4];
        float next[4][4];

        DH_Transform(&table[i], q_deg[i], link);
        for (uint8_t row = 0U; row < 4U; ++row) {
            for (uint8_t column = 0U; column < 4U; ++column) {
                next[row][column] =
                    current[row][0] * link[0][column] +
                    current[row][1] * link[1][column] +
                    current[row][2] * link[2][column] +
                    current[row][3] * link[3][column];
            }
        }
        memcpy(current, next, sizeof(current));
        memcpy(frames[i], current, sizeof(current));
    }

    shoulder = MatOriginOf(frames[0]);
    wrist = MatOriginOf(frames[4]);
    shoulder_wrist = VecSub(wrist, shoulder);
    length = VecNorm(shoulder_wrist);
    if (length < IK_ANALYTIC_MIN_LEN ||
        !ReferenceElbowAxis(param, shoulder_wrist, &elbow_axis_ref)) {
        return false;
    }

    unit = VecScale(shoulder_wrist, 1.0 / length);
    rotation_03 = MatRotationOf(frames[2]);
    elbow_axis = MatCol(rotation_03, 1U);
    dot_ref = VecDot(elbow_axis_ref, unit);
    dot_actual = VecDot(elbow_axis, unit);
    psi = atan2(VecDot(unit, VecCross(elbow_axis_ref, elbow_axis)),
                VecDot(elbow_axis_ref, elbow_axis) - dot_ref * dot_actual);
    *psi_deg = (float)(psi * IK_ANALYTIC_RAD_TO_DEG);
    return true;
}

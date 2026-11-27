#include "ik_numeric.h"

#include <math.h>
#include <string.h>

#include "position_tran.h"

#define IK_N            IK_NUMERIC_JOINTS
#define IK_DEG_TO_RAD   (3.14159265358979323846 / 180.0)
#define IK_RAD_TO_DEG   (180.0 / 3.14159265358979323846)

static double Ik_Max(double a, double b) { return a > b ? a : b; }
static double Ik_Min(double a, double b) { return a < b ? a : b; }

/* 解 A·x = b，A 对称正定（Cholesky）。a 被原地破坏。A 不正定（主元 ≤ 0 或非有限）返回 false */
static bool Ik_SolveSpd6(double a[6][6], const double b[6], double x[6])
{
    double y[6];

    for (unsigned int j = 0U; j < 6U; ++j)
    {
        double diag = a[j][j];

        for (unsigned int k = 0U; k < j; ++k)
        {
            diag -= a[j][k] * a[j][k];
        }
        if (!(diag > 1e-18) || !isfinite(diag))
        {
            return false;
        }
        a[j][j] = sqrt(diag);
        for (unsigned int i = j + 1U; i < 6U; ++i)
        {
            double sum = a[i][j];

            for (unsigned int k = 0U; k < j; ++k)
            {
                sum -= a[i][k] * a[j][k];
            }
            a[i][j] = sum / a[j][j];
        }
    }
    for (unsigned int i = 0U; i < 6U; ++i)
    {
        double sum = b[i];

        for (unsigned int k = 0U; k < i; ++k)
        {
            sum -= a[i][k] * y[k];
        }
        y[i] = sum / a[i][i];
    }
    for (int i = 5; i >= 0; --i)
    {
        double sum = y[i];

        for (int k = i + 1; k < 6; ++k)
        {
            sum -= a[k][i] * x[k];
        }
        x[i] = sum / a[i][i];
    }
    return true;
}

static double Ik_Norm3(const double *v)
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void IkNumeric_DefaultParam(IkNumericParam_t *param)
{
    param->damping = 0.03;
    param->orientation_length_m = 0.1;
    param->nullspace_gain = 0.02;
    param->max_step_deg = 0.2;
    param->max_pos_err_m = 0.01;
    param->max_rot_err_rad = 0.05;
    param->weight_floor = 1e-3;
    param->converged_pos_m = 5e-4;
    param->converged_rot_rad = 2e-3;
    param->max_limit_passes = 4;
    param->max_lead_deg = 5.0;
}

static bool Ik_Positive(double v)
{
    return isfinite(v) && v > 0.0;
}

bool IkNumeric_IsValidParam(const IkNumericParam_t *p)
{
    return p != NULL && Ik_Positive(p->damping) && Ik_Positive(p->orientation_length_m) &&
           isfinite(p->nullspace_gain) && p->nullspace_gain >= 0.0 && Ik_Positive(p->max_step_deg) &&
           Ik_Positive(p->max_pos_err_m) && Ik_Positive(p->max_rot_err_rad) && Ik_Positive(p->weight_floor) &&
           p->converged_pos_m >= 0.0 && p->converged_rot_rad >= 0.0 && p->max_limit_passes >= 0;
}

bool IkNumeric_IsValidWeights(const float weight[IK_NUMERIC_JOINTS])
{
    float sum = 0.0f;

    for (unsigned int i = 0U; i < IK_N; ++i)
    {
        if (!isfinite(weight[i]) || weight[i] < 0.0f)
        {
            return false;
        }
        sum += weight[i];
    }
    return sum > 0.0f;
}

void IkNumeric_Frames(const DH_Param_t *table, const float q_deg[IK_NUMERIC_JOINTS],
                      float frames[IK_NUMERIC_JOINTS][4][4])
{
    float link[4][4];

    for (unsigned int k = 0U; k < IK_N; ++k)
    {
        DH_Transform(&table[k], q_deg[k], link);
        if (k == 0U)
        {
            memcpy(frames[0], link, sizeof(link));
        }
        else
        {
            Trans4_Mul(frames[k - 1U], link, frames[k]);   /* 输出与输入不别名 */
        }
    }
}

void IkNumeric_Jacobian(const float frames[IK_NUMERIC_JOINTS][4][4], double jac[6][IK_NUMERIC_JOINTS])
{
    const float (*tool)[4] = frames[IK_N - 1U];

    for (unsigned int i = 0U; i < IK_N; ++i)
    {
        const float (*f)[4] = frames[i];
        const double z[3] = {f[0][2], f[1][2], f[2][2]};
        const double r[3] = {(double)tool[0][3] - f[0][3],
                             (double)tool[1][3] - f[1][3],
                             (double)tool[2][3] - f[2][3]};

        jac[0][i] = z[1] * r[2] - z[2] * r[1];
        jac[1][i] = z[2] * r[0] - z[0] * r[2];
        jac[2][i] = z[0] * r[1] - z[1] * r[0];
        jac[3][i] = z[0];
        jac[4][i] = z[1];
        jac[5][i] = z[2];
    }
}

void IkNumeric_PoseError(const float current[4][4], const float desired[4][4], double e[6])
{
    for (unsigned int k = 0U; k < 3U; ++k)
    {
        e[k] = (double)desired[k][3] - current[k][3];
        e[k + 3U] = 0.0;
    }
    for (unsigned int k = 0U; k < 3U; ++k)   /* 第 k 列：r_c × r_d */
    {
        const double c[3] = {current[0][k], current[1][k], current[2][k]};
        const double d[3] = {desired[0][k], desired[1][k], desired[2][k]};

        e[3] += 0.5 * (c[1] * d[2] - c[2] * d[1]);
        e[4] += 0.5 * (c[2] * d[0] - c[0] * d[2]);
        e[5] += 0.5 * (c[0] * d[1] - c[1] * d[0]);
    }
}

/* 自由关节集合上的加权阻尼解：dq_task = J̄e，J̄ = W⁻¹Jᵀ(J W⁻¹ Jᵀ + λ²I)⁻¹。
 * dq_null 是 z 在 J 零空间上的 W 加权投影：七个关节全自由时才有，用无阻尼伪逆求精确投影，否则为 0。
 * 不用带阻尼的 (I − J̄J)：J̄ 带阻尼时它不是投影，会把 z 漏进任务空间，远离参考构型时把任务项顶住。
 * 固定关节（is_free[i] = false）不参与，两个输出里对应位置为 0。 */
static bool Ik_SolveFreeSet(const double jac[6][IK_NUMERIC_JOINTS], const double w_inv[IK_NUMERIC_JOINTS],
                            const bool is_free[IK_NUMERIC_JOINTS], const double e[6],
                            const double z[IK_NUMERIC_JOINTS], double damping,
                            double dq_task[IK_NUMERIC_JOINTS], double dq_null[IK_NUMERIC_JOINTS])
{
    double a0[6][6];    /* J W⁻¹ Jᵀ，无阻尼，零空间投影用 */
    double a[6][6];     /* a0 + λ²I，任务项用 */
    double y[6];
    bool all_free = true;

    for (unsigned int r = 0U; r < 6U; ++r)
    {
        for (unsigned int c = 0U; c < 6U; ++c)
        {
            double sum = 0.0;

            for (unsigned int i = 0U; i < IK_N; ++i)
            {
                if (is_free[i])
                {
                    sum += w_inv[i] * jac[r][i] * jac[c][i];
                }
            }
            a0[r][c] = sum;
            a[r][c] = sum + (r == c ? damping * damping : 0.0);
        }
    }
    if (!Ik_SolveSpd6(a, e, y))//求a0 + λ²I的逆
    {
        return false;
    }
    for (unsigned int i = 0U; i < IK_N; ++i)
    {
        double task = 0.0;

        all_free = all_free && is_free[i];
        dq_null[i] = 0.0;
        if (!is_free[i])
        {
            dq_task[i] = 0.0;
            continue;
        }
        for (unsigned int r = 0U; r < 6U; ++r)
        {
            task += jac[r][i] * y[r];
        }
        dq_task[i] = w_inv[i] * task;//计算出任务空间项的dq
    }
    //零空间项改善构型
    /* 零空间项：dq_null = z − W⁻¹Jᵀ(J W⁻¹ Jᵀ)⁻¹ J z。有关节被限位固定后只剩 6 个自由关节，没有冗余度，为 0；
     * 保护：近奇异时数值放大可能使 ‖dq_null‖_W > ‖z‖_W（W 加权投影不会放大），超过时按比例缩回 */
    if (all_free)
    {
        double jz[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double u[6];

        for (unsigned int r = 0U; r < 6U; ++r)
        {
            for (unsigned int i = 0U; i < IK_N; ++i)
            {
                jz[r] += jac[r][i] * z[i];
            }
        }
        if (Ik_SolveSpd6(a0, jz, u))//J W⁻¹ Jᵀ求逆
        {
            double null_sq = 0.0;//零空间加权范数
            double z_sq = 0.0;//参考构型范数

            for (unsigned int i = 0U; i < IK_N; ++i)
            {
                double proj = 0.0;

                for (unsigned int r = 0U; r < 6U; ++r)
                {
                    proj += jac[r][i] * u[r];
                }
                dq_null[i] = z[i] - w_inv[i] * proj;//z=k(qref-q)
                null_sq += dq_null[i] * dq_null[i] / w_inv[i];
                z_sq += z[i] * z[i] / w_inv[i];
            }
            if (null_sq > z_sq)//零空间算出来的范数大于参考构型的范数，不合理按比例缩小(为啥不舍弃)
            {
                const double shrink = sqrt(z_sq / null_sq);

                for (unsigned int i = 0U; i < IK_N; ++i)
                {
                    dq_null[i] *= shrink;
                }
            }
        }
    }
    return true;
}

void IkNumeric_Step(const DH_Param_t *table, const float q_deg[IK_NUMERIC_JOINTS], const float target[4][4],
                    const float q_ref_deg[IK_NUMERIC_JOINTS], const float weight[IK_NUMERIC_JOINTS],
                    const float limit_min_deg[IK_NUMERIC_JOINTS], const float limit_max_deg[IK_NUMERIC_JOINTS],
                    const IkNumericParam_t *param, IkNumericStep_t *out)
{
    double w_inv[IK_N];//关节权重的逆
    double z[IK_N];//零空间参考构型的增量
    double lo[IK_N];//deltaq的下限
    double hi[IK_N];//deltaq的上限
    float frames[IK_N][4][4];//基座到连杆系的变换，重复了
    double jac[6][IK_N];//雅可比
    double e[6];//末端位资误差

    memset(out, 0, sizeof(*out));
    if (!IkNumeric_IsValidParam(param) || !IkNumeric_IsValidWeights(weight))
    {
        return;
    }
    for (unsigned int i = 0U; i < IK_N; ++i)
    {
        const double q = q_deg[i];

        if (!isfinite(q) || !isfinite(q_ref_deg[i]) || !(limit_min_deg[i] < limit_max_deg[i]))
        {
            return;
        }
        w_inv[i] = 1.0 / Ik_Max((double)weight[i], param->weight_floor);
        z[i] = param->nullspace_gain * ((double)q_ref_deg[i] - q) * IK_DEG_TO_RAD;
        lo[i] = Ik_Min(0.0, ((double)limit_min_deg[i] - q) * IK_DEG_TO_RAD);
        hi[i] = Ik_Max(0.0, ((double)limit_max_deg[i] - q) * IK_DEG_TO_RAD);
    }

    IkNumeric_Frames(table, q_deg, frames);//冗余计算变换矩阵
    IkNumeric_Jacobian((const float (*)[4][4])frames, jac);//冗余计算雅可比
    IkNumeric_PoseError(frames[IK_N - 1U], target, e);//计算位姿误差
    for (unsigned int r = 0U; r < 6U; ++r)
    {
        if (!isfinite(e[r]))
        {
            return;
        }
    }

    out->pos_err_m = Ik_Norm3(&e[0]);
    out->rot_err_rad = Ik_Norm3(&e[3]);
    out->converged = out->pos_err_m <= param->converged_pos_m && out->rot_err_rad <= param->converged_rot_rad;//收敛判据

    //单步误差限幅，
    {
        const double pos_scale = out->pos_err_m > param->max_pos_err_m ? param->max_pos_err_m / out->pos_err_m : 1.0;
        const double rot_scale = out->rot_err_rad > param->max_rot_err_rad ?
                                 param->max_rot_err_rad / out->rot_err_rad : 1.0;

        for (unsigned int r = 0U; r < 3U; ++r)
        {
            e[r] *= pos_scale;
            e[r + 3U] *= rot_scale * param->orientation_length_m;//比例系数
            for (unsigned int i = 0U; i < IK_N; ++i)
            {
                jac[r + 3U][i] *= param->orientation_length_m;
            }
        }
    }

    /* 限位：越界的关节钳到边界并固定，它的贡献从误差里扣掉，剩下的关节重新解 */
    {
        bool is_free[IK_N];//关节是否限位
        double fixed[IK_N] = {0.0};
        double dq_task[IK_N] = {0.0};//任务姿态增量
        double dq_null[IK_N] = {0.0};//零空间任务增量

        for (unsigned int i = 0U; i < IK_N; ++i)
        {
            is_free[i] = true;
        }
        for (int pass = 0; pass <= param->max_limit_passes; ++pass)
        {
            double residual[6];//残差
            bool violated = false;
            bool any_free = false;

            memcpy(residual, e, sizeof(residual));
            for (unsigned int i = 0U; i < IK_N; ++i)
            {
                if (!is_free[i])//到达限位后减去到达限位的关节的误差权重
                {
                    for (unsigned int r = 0U; r < 6U; ++r)
                    {
                        residual[r] -= jac[r][i] * fixed[i];
                    }
                }
            }//局部最优是否会使关节到达限位，并非最优解
            if (!Ik_SolveFreeSet(jac, w_inv, is_free, residual, z, param->damping, dq_task, dq_null))
            {
                return;
            }
            if (pass == param->max_limit_passes)
            {
                break;   /* 轮数用完：不再冻结，后面统一钳位 */
            }
            for (unsigned int i = 0U; i < IK_N; ++i)
            {
                const double total = dq_task[i] + dq_null[i];

                if (!is_free[i])
                {
                    continue;
                }
                if (total < lo[i] || total > hi[i])
                {
                    is_free[i] = false;
                    fixed[i] = total < lo[i] ? lo[i] : hi[i];
                    out->limit_mask = (uint8_t)(out->limit_mask | (1U << i));
                    violated = true;
                }
                else
                {
                    any_free = true;
                }
            }
            if (!violated || !any_free)
            {
                if (violated)   /* 全部冻结：dq 就是各自的边界值 */
                {
                    memset(dq_task, 0, sizeof(dq_task));
                    memset(dq_null, 0, sizeof(dq_null));
                }
                break;
            }
        }
        for (unsigned int i = 0U; i < IK_N; ++i)
        {
            if (!is_free[i])
            {
                dq_task[i] = fixed[i];
            }
        }

        /* 步长约束：任务项优先。任务项超步长时整体等比缩小（保持方向），零空间项只用剩下的余量 */
        {
            const double step = param->max_step_deg * IK_DEG_TO_RAD;
            double task_peak = 0.0;
            double null_scale = 1.0;

            for (unsigned int i = 0U; i < IK_N; ++i)
            {
                task_peak = Ik_Max(task_peak, fabs(dq_task[i]));
            }
            if (task_peak > step)
            {
                const double shrink = step / task_peak;

                for (unsigned int i = 0U; i < IK_N; ++i)
                {
                    dq_task[i] *= shrink;
                }
                out->step_limited = true;
            }
            for (unsigned int i = 0U; i < IK_N; ++i)
            {
                const double low = Ik_Max(-step, lo[i]);
                const double high = Ik_Min(step, hi[i]);

                if (dq_null[i] > 0.0)
                {
                    null_scale = Ik_Min(null_scale, (high - dq_task[i]) / dq_null[i]);
                }
                else if (dq_null[i] < 0.0)
                {
                    null_scale = Ik_Min(null_scale, (low - dq_task[i]) / dq_null[i]);
                }
            }
            null_scale = Ik_Max(0.0, Ik_Min(1.0, null_scale));

            for (unsigned int i = 0U; i < IK_N; ++i)
            {
                const double dq = Ik_Max(lo[i], Ik_Min(hi[i], dq_task[i] + null_scale * dq_null[i]));

                out->dq_deg[i] = (float)(dq * IK_RAD_TO_DEG);
            }
        }
    }
    out->ok = true;
}

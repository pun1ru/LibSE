#include "arm_dynamics.h"

#include <math.h>
#include <string.h>

#include "position_tran.h"

/* 算法：关节 j 的重力矩 tau_j = g * sum_{i>=j} sz^T * T_{j-1} * dA_j * A_{j+1}...A_i * pi_i
 * 只用到结果矩阵的第 3 行（sz 选 z），而 M * A_k 的第 3 行只取决于 M 的第 3 行：
 *     row2(M * A) = row2(M) * A
 * 所以不必维护完整 4x4，只递推一个 1x4 行向量 v：
 *     v0 = row2(T_{j-1}) * dA_j          (T_{j-1} 的第 3 行逐关节右乘 A 得到)
 *     v  = v * A_{i}  (i = j+1 ... 6)
 *     tau_j += g * (v . pi_i)            (pi 版本：4 项；beta 版本：只有 x、y 两项)
 * 与文档里 M = T[j]*dA[j]; M = M*A[i] 的 4x4 递推逐项等价，只是少算了用不到的 3 行
 *
 * dA_j 直接由 A_j 得到，不再多调一次 sin/cos：A 的前两列是 [c; s*ca; s*sa] 与 [-s; c*ca; c*sa]，
 * 对 q 求导就是把它们互换并取负，后两列与 q 无关，导数为 0
 */

/* 角度 deg -> rad 的换算在 DH_Transform 里做，与正运动学共用同一份，这里不重复
 * 静态工作区：约 7*64 + 7*64 + 2*16 字节，放 .bss，不占 ArmTask 的栈（TASK_ARM_STACK_SIZE 只有 256 words） */
static float s_A[ARM_GRAVITY_DOF][4][4];        /* A[k] = T_{k,k+1}，DH 一行一个 */
static float s_row2[ARM_GRAVITY_DOF + 1U][4];   /* s_row2[k] = T_{0..k-1} 的第 3 行；k=0 即 [0 0 1 0] */

/* out = in * M（1x4 行向量乘 4x4），out 与 in 不能别名 */
static inline void RowMulMat(const float in[4], const float M[4][4], float out[4])
{
    for (unsigned int c = 0U; c < 4U; ++c)
    {
        out[c] = in[0] * M[0][c] + in[1] * M[1][c] + in[2] * M[2][c] + in[3] * M[3][c];
    }
}

/* v0 = row2(T_{j-1}) * dA_j；dA_j 的第 3、4 列恒为 0 */
static inline void RowMulDA(const float row[4], const float A[4][4], float out[4])
{
    /* A 的列 0 = [c, s*ca, s*sa, 0]^T，列 1 = [-s, c*ca, c*sa, 0]^T
     * dA 的列 0 = [-s, c*ca, c*sa, 0]^T = A 的列 1；dA 的列 1 = [-c, -s*ca, -s*sa, 0]^T = -(A 的列 0) */
    out[0] = row[0] * A[0][1] + row[1] * A[1][1] + row[2] * A[2][1] + row[3] * A[3][1];
    out[1] = -(row[0] * A[0][0] + row[1] * A[1][0] + row[2] * A[2][0] + row[3] * A[3][0]);
    out[2] = 0.0f;
    out[3] = 0.0f;
}

/* 准备静态工作区：每行 DH 的变换 A[k]，以及 T 第 3 行的前缀积 */
static void PrepareChain(const DH_Param_t *dh, const float *q_deg)
{
    static const float z_row[4] = {0.0f, 0.0f, 1.0f, 0.0f};

    for (uint8_t k = 0U; k < ARM_GRAVITY_DOF; ++k)
    {
        DH_Transform(&dh[k], q_deg[k], s_A[k]);
    }
    memcpy(s_row2[0], z_row, sizeof(z_row));
    for (uint8_t k = 0U; k < ARM_GRAVITY_DOF; ++k)
    {
        RowMulMat(s_row2[k], s_A[k], s_row2[k + 1U]);
    }
}

void ArmDynamics_GravityTorquePi(const DH_Param_t *dh, const float *q_deg, const float *pi,
                                 float g, float *tau)
{
    float v[4];
    float w[4];

    PrepareChain(dh, q_deg);
    for (uint8_t j = 0U; j < ARM_GRAVITY_DOF; ++j)
    {
        float sum = 0.0f;

        RowMulDA(s_row2[j], s_A[j], v);
        for (uint8_t i = j; i < ARM_GRAVITY_DOF; ++i)
        {
            const float *p = &pi[4U * i];

            if (i > j)
            {
                RowMulMat(v, s_A[i], w);
                memcpy(v, w, sizeof(v));
            }
            sum += v[0] * p[0] + v[1] * p[1] + v[2] * p[2] + v[3] * p[3];
        }
        tau[j] = g * sum;
    }
}

void ArmDynamics_GravityTorqueBeta(const DH_Param_t *dh, const float *q_deg, const float *beta,
                                   float g, float *tau)
{
    float v[4];
    float w[4];

    PrepareChain(dh, q_deg);
    for (uint8_t j = 0U; j < ARM_GRAVITY_DOF; ++j)
    {
        float sum = 0.0f;

        RowMulDA(s_row2[j], s_A[j], v);
        for (uint8_t i = j; i < ARM_GRAVITY_DOF; ++i)
        {
            if (i > j)
            {
                RowMulMat(v, s_A[i], w);
                memcpy(v, w, sizeof(v));
            }
            /* beta 只含连杆 1..6（连杆 0 的参数不进回归），每连杆 2 个：hx、hy */
            if (i >= 1U)
            {
                sum += v[0] * beta[2U * (i - 1U)] + v[1] * beta[2U * (i - 1U) + 1U];
            }
        }
        tau[j] = g * sum;
    }
}

bool ArmDynamics_GravityModel(const ArmGravityParam_t *param, const DH_Param_t *dh,
                              const float *q_deg, float *tau)
{
    float g;
    const float *table;

    memset(tau, 0, ARM_GRAVITY_DOF * sizeof(float));
    if (param == NULL || dh == NULL || q_deg == NULL || param->enable == 0U)
    {
        return false;
    }
    table = param->use_beta != 0U ? param->beta : param->pi;
    if (table == NULL)
    {
        return false;
    }
    /* 反馈还没来或数据异常时 q 可能是 NaN/Inf，sinf/cosf 会把它一路带进力矩；只在这里查一次
     * 用 x - x == 0 判有限：NaN/Inf 时结果是 NaN，比较为假；不依赖 isfinite 宏，对编译器的快速数学选项也稳 */
    for (uint8_t k = 0U; k < ARM_GRAVITY_DOF; ++k)
    {
        if (!((q_deg[k] - q_deg[k]) == 0.0f))
        {
            return false;
        }
    }

    g = param->g > 0.0f ? param->g : ARM_GRAVITY_DEFAULT_G;
    if (param->use_beta != 0U)
    {
        ArmDynamics_GravityTorqueBeta(dh, q_deg, table, g, tau);
    }
    else
    {
        ArmDynamics_GravityTorquePi(dh, q_deg, table, g, tau);
    }
    return true;
}

void ArmDynamics_GravityCommand(const float *model, bool valid, int8_t sign, const float *limit,
                                uint8_t mask, float *cmd)
{
    /* sign 未配（0）按 +1：避免忘记配置时变成零前馈而难以发现；符号只在这里乘一次，不要在别处再取反 */
    const float s = sign < 0 ? -1.0f : 1.0f;

    for (uint8_t i = 0U; i < ARM_GRAVITY_DOF; ++i)
    {
        float tau = s * model[i];

        /* 无效 / 该轴被掩码关掉 / 值非有限：不输出 */
        if (!valid || ((mask >> i) & 1U) == 0U || !((tau - tau) == 0.0f))
        {
            tau = 0.0f;
        }
        else if (limit != NULL)
        {
            const float lim = limit[i];

            if (!(lim > 0.0f))
            {
                tau = 0.0f;
            }
            else if (tau > lim)
            {
                tau = lim;
            }
            else if (tau < -lim)
            {
                tau = -lim;
            }
        }
        cmd[i] = tau;
    }
}

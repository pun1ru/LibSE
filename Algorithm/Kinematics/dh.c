#include "dh.h"

#include <math.h>
#include <string.h>

#include "arm_matrix.h"

#define DH_DEG_TO_RAD (3.141592653589793f / 180.0f)

void DH_Transform(const DH_Param_t *param, float q_deg, float T[4][4])
{
    float theta = (q_deg + param->theta_offset_deg) * DH_DEG_TO_RAD;
    float alpha = param->alpha_deg * DH_DEG_TO_RAD;
    float ct = cosf(theta), st = sinf(theta);
    float ca = cosf(alpha), sa = sinf(alpha);

    T[0][0] = ct;      T[0][1] = -st;      T[0][2] = 0.0f; T[0][3] = param->a_m;
    T[1][0] = st * ca; T[1][1] = ct * ca;  T[1][2] = -sa;  T[1][3] = -sa * param->d_m;
    T[2][0] = st * sa; T[2][1] = ct * sa;  T[2][2] = ca;   T[2][3] = ca * param->d_m;
    T[3][0] = 0.0f;    T[3][1] = 0.0f;     T[3][2] = 0.0f; T[3][3] = 1.0f;
}

void DH_ForwardKinematics(const DH_Param_t *table, const float *q_deg,
                          uint8_t n, float T[4][4])
{
    float joint[4][4];
    float result[4][4];
    matrix_t mat_T, mat_joint, mat_result;

    matrix_init(&mat_T, 4, 4, &T[0][0]);
    matrix_init(&mat_joint, 4, 4, &joint[0][0]);
    matrix_init(&mat_result, 4, 4, &result[0][0]);

    memset(T, 0, sizeof(float) * 16U);
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        T[i][i] = 1.0f;
    }

    /* arm_mat_mult_f32 不支持输出与输入重叠，先乘到 result 再拷回 */
    for (uint8_t k = 0U; k < n; ++k)
    {
        DH_Transform(&table[k], q_deg[k], joint);
        matrix_mult(&mat_T, &mat_joint, &mat_result);
        memcpy(T, result, sizeof(result));
    }
}

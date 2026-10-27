#include "dh.h"

#include <math.h>
#include <string.h>

#include "position_tran.h"

#define DH_DEG_TO_RAD (3.141592653589793f / 180.0f)
//根据构建的dh参数构建变换矩阵
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
//根据dh参数表返回对应的变换矩阵数组
void DH_LinkTransforms(const DH_Param_t *table, const float *q_deg, uint8_t n, float (*T)[4][4])
{
    for (uint8_t k = 0U; k < n; ++k)
    {
        DH_Transform(&table[k], q_deg[k], T[k]);
    }
}
//正向运动学
void DH_ForwardKinematics(const DH_Param_t *table, const float *q_deg,
                          uint8_t n, float T[4][4])
{
    float joint[4][4];
    float result[4][4];

    Trans4_Identity(T);

    /* Trans4_Mul 不支持输出与输入重叠，先乘到 result 再拷回 */
    for (uint8_t k = 0U; k < n; ++k)
    {
        DH_Transform(&table[k], q_deg[k], joint);
        Trans4_Mul(T, joint, result);
        memcpy(T, result, sizeof(result));
    }
}

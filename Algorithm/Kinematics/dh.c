#include "dh.h"

#include <math.h>
#include <string.h>

void DH_Transform(const DH_Param_t *param, float q_rad, float T[4][4])
{
    float theta = q_rad + param->theta_offset_rad;
    float ct = cosf(theta), st = sinf(theta);
    float ca = cosf(param->alpha_rad), sa = sinf(param->alpha_rad);

    T[0][0] = ct;   T[0][1] = -st * ca; T[0][2] = st * sa;  T[0][3] = param->a_m * ct;
    T[1][0] = st;   T[1][1] = ct * ca;  T[1][2] = -ct * sa; T[1][3] = param->a_m * st;
    T[2][0] = 0.0f; T[2][1] = sa;       T[2][2] = ca;       T[2][3] = param->d_m;
    T[3][0] = 0.0f; T[3][1] = 0.0f;     T[3][2] = 0.0f;     T[3][3] = 1.0f;
}

void DH_ForwardKinematics(const DH_Param_t *table, const float *q_rad,
                          uint8_t n, float T[4][4])
{
    float joint[4][4];
    float result[4][4];

    memset(T, 0, sizeof(float) * 16U);
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        T[i][i] = 1.0f;
    }

    for (uint8_t k = 0U; k < n; ++k)
    {
        DH_Transform(&table[k], q_rad[k], joint);
        for (uint8_t r = 0U; r < 4U; ++r)
        {
            for (uint8_t c = 0U; c < 4U; ++c)
            {
                result[r][c] = T[r][0] * joint[0][c] + T[r][1] * joint[1][c] +
                               T[r][2] * joint[2][c] + T[r][3] * joint[3][c];
            }
        }
        memcpy(T, result, sizeof(result));
    }
}

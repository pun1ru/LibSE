#include "kinematics.h"

void Kin_TwistToParent(const float T[4][4], const Twist_t *child, Twist_t *parent)
{
    float pos[3];
    float omega[3];
    float vel[3];
    float shift[3];

    Trans4_GetPos(T, pos);
    Trans4_MulVec(T, child->angular, omega);
    Trans4_MulVec(T, child->linear, vel);
    Vec3_Cross(pos, omega, shift);

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        parent->linear[i] = vel[i] + shift[i];
        parent->angular[i] = omega[i];
    }
}

void Kin_TwistToChild(const float T[4][4], const Twist_t *parent, Twist_t *child)
{
    float pos[3];
    float shift[3];
    float vel_sum[3];
    float vel[3];
    float omega[3];

    Trans4_GetPos(T, pos);
    Vec3_Cross(parent->angular, pos, shift);
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        vel_sum[i] = parent->linear[i] + shift[i];
    }
    Trans4_TransMulVec(T, vel_sum, vel);
    Trans4_TransMulVec(T, parent->angular, omega);

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        child->linear[i] = vel[i];
        child->angular[i] = omega[i];
    }
}

void Kin_PropagateRevolute(const float T[4][4], const Twist_t *parent, float joint_rate,
                           Twist_t *child)
{
    Kin_TwistToChild(T, parent, child);
    child->angular[KIN_AXIS_Z] += joint_rate;
}

void Kin_PropagateAngular(const float T[4][4], const float parent[3], float joint_rate,
                          float child[3])
{
    float omega[3];

    Trans4_TransMulVec(T, parent, omega);
    omega[KIN_AXIS_Z] += joint_rate;
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        child[i] = omega[i];
    }
}

void Kin_ChainAngular(const float (*T)[4][4], const float *rate, uint8_t n, const float base[3],
                      float (*out)[3])
{
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        out[0][i] = base[i];
    }
    for (uint8_t k = 0U; k < n; ++k)
    {
        Kin_PropagateAngular(T[k], out[k], rate[k], out[k + 1U]);
    }
}

void Kin_ChainTwist(const float (*T)[4][4], const float *rate, uint8_t n, const Twist_t *base,
                    Twist_t *out)
{
    out[0] = *base;
    for (uint8_t k = 0U; k < n; ++k)
    {
        Kin_PropagateRevolute(T[k], &out[k], rate[k], &out[k + 1U]);
    }
}

void Kin_ChainAngularInBase(const float (*T)[4][4], const float *rate, uint8_t n,
                            const float base[3], float (*out)[3])
{
    float rot[3][3];
    float link_rot[3][3];
    float next[3][3];

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        out[0][i] = base[i];
        for (unsigned int j = 0U; j < 3U; ++j)
        {
            rot[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    for (uint8_t k = 0U; k < n; ++k)
    {
        Trans4_GetRotation(T[k], link_rot);
        Mat3_Mul((const float (*)[3])rot, (const float (*)[3])link_rot, next);
        for (unsigned int i = 0U; i < 3U; ++i)
        {
            out[k + 1U][i] = out[k][i] + rate[k] * next[i][KIN_AXIS_Z];
            for (unsigned int j = 0U; j < 3U; ++j)
            {
                rot[i][j] = next[i][j];
            }
        }
    }
}

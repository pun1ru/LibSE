#include "dynamics.h"

#include <stddef.h>

void Dyn_PropagateAccel(const float T[4][4], const LinkMotion_t *parent, float joint_rate,
                        float joint_accel, LinkMotion_t *child)
{
    float pos[3];
    float omega[3];
    float axis_rate[3] = {0.0f, 0.0f, 0.0f};
    float coriolis[3];
    float alpha[3];
    float alpha_cross_pos[3];
    float omega_cross_pos[3];
    float centripetal[3];
    float lin_parent[3];
    float lin[3];
    Twist_t twist;

    Trans4_GetPos(T, pos);

    /* 角加速度：R^T alpha_p + (R^T w_p) x (rate z) + accel z */
    axis_rate[KIN_AXIS_Z] = joint_rate;
    Trans4_TransMulVec(T, parent->twist.angular, omega);
    Trans4_TransMulVec(T, parent->accel.angular, alpha);
    Vec3_Cross(omega, axis_rate, coriolis);
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        alpha[i] += coriolis[i];
    }
    alpha[KIN_AXIS_Z] += joint_accel;

    /* 线加速度：R^T (alpha_p x p + w_p x (w_p x p) + a_p) */
    Vec3_Cross(parent->accel.angular, pos, alpha_cross_pos);
    Vec3_Cross(parent->twist.angular, pos, omega_cross_pos);
    Vec3_Cross(parent->twist.angular, omega_cross_pos, centripetal);
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        lin_parent[i] = alpha_cross_pos[i] + centripetal[i] + parent->accel.linear[i];
    }
    Trans4_TransMulVec(T, lin_parent, lin);

    Kin_PropagateRevolute(T, &parent->twist, joint_rate, &twist);

    child->twist = twist;
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        child->accel.angular[i] = alpha[i];
        child->accel.linear[i] = lin[i];
    }
}

void Dyn_WrenchToParent(const float T[4][4], const Wrench_t *child, Wrench_t *parent)
{
    float pos[3];
    float force[3];
    float moment[3];
    float shift[3];

    Trans4_GetPos(T, pos);
    Trans4_MulVec(T, child->force, force);
    Trans4_MulVec(T, child->moment, moment);
    Vec3_Cross(pos, force, shift);

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        parent->force[i] = force[i];
        parent->moment[i] = moment[i] + shift[i];
    }
}

void Dyn_WrenchToChild(const float T[4][4], const Wrench_t *parent, Wrench_t *child)
{
    float pos[3];
    float shift[3];
    float moment_origin[3];
    float force[3];
    float moment[3];

    Trans4_GetPos(T, pos);
    Vec3_Cross(pos, parent->force, shift);
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        moment_origin[i] = parent->moment[i] - shift[i];
    }
    Trans4_TransMulVec(T, parent->force, force);
    Trans4_TransMulVec(T, moment_origin, moment);

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        child->force[i] = force[i];
        child->moment[i] = moment[i];
    }
}

void Dyn_PropagateWrench(const float T[4][4], const LinkInertia_t *link,
                         const LinkMotion_t *motion, const Wrench_t *next, Wrench_t *out)
{
    float alpha_cross_com[3];
    float omega_cross_com[3];
    float centripetal[3];
    float inertial_force[3];
    float inertia_alpha[3];
    float inertia_omega[3];
    float gyro[3];
    float moment_com[3];
    float moment_origin[3];
    Wrench_t carried;

    /* 质心加速度 a_c = a + alpha x c + w x (w x c)，惯性力 F = m a_c */
    Vec3_Cross(motion->accel.angular, link->com_m, alpha_cross_com);
    Vec3_Cross(motion->twist.angular, link->com_m, omega_cross_com);
    Vec3_Cross(motion->twist.angular, omega_cross_com, centripetal);
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        inertial_force[i] = link->mass_kg * (motion->accel.linear[i] + alpha_cross_com[i] +
                                             centripetal[i]);
    }

    /* 惯性力矩 N = I alpha + w x (I w)，再搬到连杆原点：n = N + c x F */
    Mat3_MulVec(link->inertia, motion->accel.angular, inertia_alpha);
    Mat3_MulVec(link->inertia, motion->twist.angular, inertia_omega);
    Vec3_Cross(motion->twist.angular, inertia_omega, gyro);
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        moment_com[i] = inertia_alpha[i] + gyro[i];
    }
    Vec3_Cross(link->com_m, inertial_force, moment_origin);

    Dyn_WrenchToParent(T, next, &carried);
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        out->force[i] = carried.force[i] + inertial_force[i];
        out->moment[i] = carried.moment[i] + moment_com[i] + moment_origin[i];
    }
}

void Dyn_ChainAccel(const DynChain_t *chain, const LinkMotion_t *base, LinkMotion_t *motion)
{
    motion[0] = *base;
    for (uint8_t k = 0U; k < chain->n; ++k)
    {
        Dyn_PropagateAccel(chain->T[k], &motion[k], chain->rate[k], chain->accel[k],
                           &motion[k + 1U]);
    }
}

void Dyn_InverseDynamics(const DynChain_t *chain, const LinkMotion_t *base, const Wrench_t *tip,
                         LinkMotion_t *motion, float *torque)
{
    float tip_identity[4][4];
    Wrench_t wrench = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

    Dyn_ChainAccel(chain, base, motion);
    if (tip != NULL)
    {
        wrench = *tip;
    }

    /* 末端连杆的 next 就是 {n} 系下的外力，坐标变换取单位阵 */
    Trans4_Identity(tip_identity);
    for (uint8_t k = chain->n; k >= 1U; --k)
    {
        if (k == chain->n)
        {
            Dyn_PropagateWrench(tip_identity, &chain->link[k - 1U], &motion[k], &wrench, &wrench);
        }
        else
        {
            Dyn_PropagateWrench(chain->T[k], &chain->link[k - 1U], &motion[k], &wrench, &wrench);
        }
        torque[k - 1U] = Dyn_JointTorque(&wrench);
    }
}

void Dyn_InertiaToFrame(const float T[4][4], const LinkInertia_t *link, float out[3][3])
{
    float rotation[3][3];
    float rotated[3][3];
    float com[3];
    float com_sq = 0.0f;

    Trans4_GetRotation(T, rotation);
    Trans4_MulPoint(T, link->com_m, com);
    Mat3_Mul(rotation, link->inertia, rotated);
    Mat3_MulT(rotated, rotation, out);

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        com_sq += com[i] * com[i];
    }
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        for (unsigned int j = 0U; j < 3U; ++j)
        {
            float delta = (i == j) ? com_sq : 0.0f;
            out[i][j] += link->mass_kg * (delta - com[i] * com[j]);
        }
    }
}

#ifndef ARM_DYNAMICS_H
#define ARM_DYNAMICS_H

#include <stdbool.h>
#include <stdint.h>

#include "dh.h"

/* 七轴臂重力矩在线计算，tau_g = Y(q) * pi，推导见仓库根目录「重力补偿推导.md」「重力补偿符号约定与Yq说明.md」
 * 本文件只放纯算法：输入 DH 表、关节角（deg）、参数向量，输出各关节重力矩 N*m，不依赖 Arm_t / 电机 / FreeRTOS，
 * 所以同一份 C 源码可以直接在主机上编译，与上位机 Python 的 gravity_regressor() 逐点对拍
 *
 * 方向约定：输出就是 dU/dq，即关节为保持姿态必须输出的力矩；与电机指令是否同号由 ArmGravityParam_t.sign 决定，
 * 符号只在 Arm_GravityCommand 里乘一次，这里的函数一律不带符号
 *
 * 参数向量有两套，数值上等价（同一组 pi 投影到 beta 后在一般位形下误差为机器精度）：
 *   28 维 pi  = [h1x h1y h1z m1, ... h7x h7y h7z m7]，h = m*c，连杆下标 0..6 对应 J0..J6，用于核对已有辨识结果
 *   12 维 beta = [h2x h2y h3x h3y h4x h4y h5x h5y h6x h6y h7x h7y]，对应 pi 的列 ARM_GRAVITY_BETA_SEL，用于正式控制
 */
#define ARM_GRAVITY_DOF         7U      /* 必须等于 ARM_JOINT_COUNT，arm.c 里有静态断言 */
#define ARM_GRAVITY_PI_DIM      (4U * ARM_GRAVITY_DOF)      /* 28 */
#define ARM_GRAVITY_BETA_DIM    12U
#define ARM_GRAVITY_DEFAULT_G   9.81f   /* 与上位机 gravity_fit.py 的 G 保持一致，不要换成 9.80665 */
#define ARM_GRAVITY_ALL_JOINTS  ((1U << ARM_GRAVITY_DOF) - 1U)     /* joint_mask 全开 */

/* beta[k] 对应 pi 的哪一列（0 基），顺序即 beta 的顺序 */
#define ARM_GRAVITY_BETA_SEL    {4U, 5U, 8U, 9U, 12U, 13U, 16U, 17U, 20U, 21U, 24U, 25U}

typedef struct {
    uint8_t enable;     /* 0：不计算，重力矩恒为 0，也不会有任何前馈输出 */
    uint8_t use_beta;   /* 1：用 12 维 beta（正式控制）；0：用 28 维 pi（核对用） */
    int8_t sign;        /* 重力前馈相对模型 dU/dq 的符号：+1 同号（默认）；-1 反号；0 按 +1 处理 */
    uint8_t joint_mask; /* bit i = 1：J_i 的前馈指令有效；0：该轴不补偿（torque_cmd 为 0，torque_model 仍计算）
                         * 注意结构体整体清零时为 0 = 全部不补偿，所以必须显式给 ARM_GRAVITY_ALL_JOINTS 或具体掩码 */
    float g;            /* 重力加速度 m/s^2，0 按 ARM_GRAVITY_DEFAULT_G 处理 */
    /* 参数表指针：默认指向 module_init.c 里的 const 表（在 Flash），要换成 RAM 里的表只改这里的指针
     * 表长度由 use_beta 决定，NULL 时当作没有参数，不输出 */
    const float *pi;            /* ARM_GRAVITY_PI_DIM 个 */
    const float *beta;          /* ARM_GRAVITY_BETA_DIM 个 */
} ArmGravityParam_t;

/**
 * \brief 28 维 pi 版本，按关节行直接累加，不生成 7x28 矩阵
 * \param[in]  dh     DH 表（改进 DH，ARM_GRAVITY_DOF 行）
 * \param[in]  q_deg  各关节角 deg，ARM_GRAVITY_DOF 个，内部转 rad
 * \param[in]  pi     28 维参数
 * \param[in]  g      重力加速度
 * \param[out] tau    各关节重力矩 N*m，ARM_GRAVITY_DOF 个
 * \note   用到的中间量全部放在文件内 static 工作区（不占调用任务的栈），所以不可重入：
 *         只能在同一个任务里调用，不要在中断或第二个任务里再调
 */
void ArmDynamics_GravityTorquePi(const DH_Param_t *dh, const float *q_deg, const float *pi,
                                 float g, float *tau);

/** \brief 12 维 beta 版本，参数含义、约束同 ArmDynamics_GravityTorquePi */
void ArmDynamics_GravityTorqueBeta(const DH_Param_t *dh, const float *q_deg, const float *beta,
                                   float g, float *tau);

/**
 * \brief 按参数结构选 pi / beta 计算原始重力矩（不乘符号、不限幅）
 * \return true 正常；false：未使能 / 参数表为空 / 输入含 NaN 或 Inf，此时 tau 已清零
 */
bool ArmDynamics_GravityModel(const ArmGravityParam_t *param, const DH_Param_t *dh,
                              const float *q_deg, float *tau);

/**
 * \brief 模型力矩 -> 前馈指令：乘符号（只在这里乘一次），再按各关节力矩限幅截断
 * \param[in]  model  ArmDynamics_GravityModel 的输出，ARM_GRAVITY_DOF 个
 * \param[in]  valid  ArmDynamics_GravityModel 的返回值，false 时全部输出 0
 * \param[in]  sign   <0 取 -1，其余取 +1
 * \param[in]  limit  各关节独立限幅 N*m，ARM_GRAVITY_DOF 个；<=0 或 NaN 的关节输出 0
 *                    NULL 表示不做单轴预限幅，由调用方在叠加 PID 后统一做总力矩限幅
 * \param[in]  mask   joint_mask：bit i = 0 的关节输出 0
 * \param[out] cmd    前馈指令；model 里有 NaN/Inf 的关节也输出 0，不会把坏值带给电机
 */
void ArmDynamics_GravityCommand(const float *model, bool valid, int8_t sign, const float *limit,
                                uint8_t mask, float *cmd);

#endif

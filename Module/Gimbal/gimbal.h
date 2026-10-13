#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

#include "Motor.h"
#include "algorism.h"
#include "dh.h"
#include "ins.h"
#include "pid.h"

/* 角度统一 rad，角速度 rad/s，角加速度 rad/s^2
 * 每个控制周期依次调用：Gimbal_Input -> Gimbal_Estimate -> Gimbal_Control
 */

/*--------------------------------------状态机--------------------------------------*/
typedef enum {
    GIMBAL_PROTECTED,   ///< 保护：输出清零，目标跟随观测
    GIMBAL_NORMAL,      ///< 常规跟随
    GIMBAL_SNIPER       ///< 吊射：换一套更硬的 PID
} GimbalState_t;

typedef enum {
    GIMBAL_YAW = 0,
    GIMBAL_PITCH,
    GIMBAL_AXIS_COUNT
} GimbalAxis_t;

/*--------------------------------------物理模型--------------------------------------*/
typedef struct {
    DH_Param_t dh[GIMBAL_AXIS_COUNT];           ///< DH 参数表，顺序 yaw -> pitch
    float mass_kg[GIMBAL_AXIS_COUNT];
    float com_m[GIMBAL_AXIS_COUNT][3];          ///< 质心在连杆坐标系下的位置 [x, y, z]
} GimbalModel_t;

/*--------------------------------------输入层--------------------------------------*/
/* 上层（遥控/自瞄）给出的原始指令 */
typedef struct {
    GimbalState_t state;
    float pitch_rad;
    float yaw_rad;
    float pitch_omega_rads;
    float yaw_omega_rads;
} GimbalInput_t;

/* 输入层处理后的目标 */
typedef struct {
    float pitch_rad;
    float yaw_rad;
    float pitch_omega_rads; ///< 期望角速度，作为速度环前馈
    float yaw_omega_rads;
} GimbalTarget_t;

/*--------------------------------------估计层--------------------------------------*/
typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float pitch_omega_rads;     ///< 欧拉角一阶导
    float yaw_omega_rads;
    float pitch_alpha_rads2;    ///< 欧拉角二阶导（差分 + SmoothFilter）
    float yaw_alpha_rads2;
    float yaw_motor_angle_rad;  ///< 电机编码器单圈角 [-pi, pi]
    float pitch_motor_angle_rad;
    uint8_t valid;              ///< INS 就绪且本周期取到数据
    SmoothFilter pitch_alpha_filter;
    SmoothFilter yaw_alpha_filter;
} GimbalObserver_t;

/*--------------------------------------控制层--------------------------------------*/
typedef struct {
    float kp;
    float ki;
    float kd;
    float sum_error_max;
    float output_max;
} GimbalPIDParam_t;

typedef struct {
    GimbalPIDParam_t angle;     ///< 角度环：rad -> rad/s
    GimbalPIDParam_t speed;     ///< 速度环：rad/s -> 电机指令
} GimbalAxisParam_t;

typedef struct {
    GimbalAxisParam_t axis[GIMBAL_AXIS_COUNT];
} GimbalGainSet_t;

typedef struct {
    PIDStruct angle_pid;
    PIDStruct speed_pid;
    float speed_ref_rads;       ///< 角度环输出 + 目标角速度前馈
    float feedforward;          ///< 模型前馈（重力补偿等），暂为 0
    float output;               ///< 写入电机的最终指令
} GimbalAxisCtrl_t;

typedef struct {
    GimbalAxisCtrl_t axis[GIMBAL_AXIS_COUNT];
} GimbalController_t;

/*--------------------------------------对象与构造参数--------------------------------------*/
/* 电机方向用电机自身的 inverse 配置：正指令应让对应欧拉角增大 */
typedef struct {
    GimbalModel_t model;
    Motor_t *yaw_motor;
    Motor_t *pitch_motor;
    GimbalGainSet_t normal_gain;
    GimbalGainSet_t sniper_gain;
    const INS_t *ins;           ///< 安装在 pitch 连杆上的 IMU
    float pitch_min_rad;
    float pitch_max_rad;
    float alpha_filter;         ///< 二阶导低通系数 0~1，越大越平滑
    GimbalState_t init_state;
} GimbalConfig_t;

typedef struct {
    GimbalState_t state;
    GimbalModel_t model;
    Motor_t *motor[GIMBAL_AXIS_COUNT];
    GimbalTarget_t target;
    GimbalObserver_t observer;
    GimbalController_t controller;
    GimbalGainSet_t normal_gain;
    GimbalGainSet_t sniper_gain;
    const INS_t *ins;
    float pitch_min_rad;
    float pitch_max_rad;
} Gimbal_t;

void Gimbal_Init(Gimbal_t *gimbal, const GimbalConfig_t *config);
/* 输入层：状态切换，目标限幅（pitch）与折算（yaw）
 * 保护状态下忽略外部角度，目标由控制层跟随当前姿态；
 * 上层做增量控制时应以 gimbal->target 为基准，切回 NORMAL 不会跳变 */
void Gimbal_Input(Gimbal_t *gimbal, const GimbalInput_t *input);
/* 估计层：从 INS 和电机反馈更新 observer */
void Gimbal_Estimate(Gimbal_t *gimbal, float dt_s);
/* 控制层：双环 PID + 前馈，结果写入两个电机的 motorcmd，不负责发送 */
void Gimbal_Control(Gimbal_t *gimbal);

#endif

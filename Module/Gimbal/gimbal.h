#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

#include "Motor.h"
#include "algorism.h"
#include "dh.h"
#include "ins.h"
#include "pid.h"

/* 角度统一 deg，角速度 deg/s
 * 每个控制周期依次调用：Gimbal_Input -> Gimbal_Estimate -> Gimbal_Control
 * 坐标系与关节角定义见 gimbal.md：底盘系与末端系均为 X 前、Y 左、Z 上
 * 世界系目标与观测沿用 INS 约定：yaw 顺时针为正，pitch 抬头为正
 * 关节角：yaw 绕底盘 Z 轴逆时针为正，pitch 抬头为正，零位时枪管水平朝底盘正前方
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

/* DH 坐标系：两个关节坐标系 + 末端云台坐标系（固定，不对应关节） */
typedef enum {
    GIMBAL_FRAME_YAW = 0,   ///< {J1}：z 沿 yaw 轴
    GIMBAL_FRAME_PITCH,     ///< {J2}：z 沿 pitch 轴，x 沿枪管
    GIMBAL_FRAME_END,       ///< {G}：末端云台系，X 前 Y 左 Z 上，与 IMU 对齐
    GIMBAL_FRAME_COUNT
} GimbalFrame_t;

/*--------------------------------------物理模型--------------------------------------*/
/* DH 表按 gimbal.md 填写（改进 DH），旋转部分须满足 R03 = Rz(yaw) * Ry(-pitch)：
 * yaw   | α=0   | a=0  | d=0 | θ=faiz
 * pitch | α=90  | a=-l | d=0 | θ=theta
 * end   | α=-90 | a=L  | d=0 | θ=0
 * 质量参数只属于两个运动连杆，连杆 i 的质心表示在关节坐标系 i 下 */
typedef struct {
    DH_Param_t dh[GIMBAL_FRAME_COUNT];          ///< DH 参数表，顺序 yaw -> pitch -> end
    float mass_kg[GIMBAL_AXIS_COUNT];
    float com_m[GIMBAL_AXIS_COUNT][3];          ///< 质心在关节坐标系下的位置 [x, y, z]
} GimbalModel_t;

/*--------------------------------------输入层--------------------------------------*/
/* 上层（遥控/自瞄）给出的世界系指向，INS 约定 */
typedef struct {
    GimbalState_t state;
    float pitch_deg;
    float yaw_deg;
    float pitch_omega_dps;
    float yaw_omega_dps;
} GimbalInput_t;

/* 输入层处理后的世界系目标 */
typedef struct {
    float pitch_deg;
    float yaw_deg;
    float pitch_omega_dps; ///< 期望世界角速度，映射到关节后作为速度环前馈
    float yaw_omega_dps;
} GimbalTarget_t;

/*--------------------------------------估计层--------------------------------------*/
typedef struct {
    float roll_deg;                             ///< 末端世界姿态，INS 约定
    float pitch_deg;
    float yaw_deg;
    float joint_deg[GIMBAL_AXIS_COUNT];         ///< 编码器关节角，方向按 motor->inverse 折算
    float joint_omega_dps[GIMBAL_AXIS_COUNT];   ///< 编码器关节角速度（相对底盘）
    float gimbal_to_world[3][3];                ///< 末端 -> 世界，由 INS 欧拉角构造
    float gimbal_to_base[3][3];                 ///< 末端 -> 底盘，由 DH 正运动学构造
    float base_to_world[3][3];                  ///< 底盘 -> 世界，由前两者反解
    float world_to_base[3][3];                  ///< base_to_world 的转置，用于把目标投到底盘系
    uint8_t valid;                              ///< INS 就绪且本周期取到数据
} GimbalObserver_t;

/*--------------------------------------控制层--------------------------------------*/
/* D 项按每周期误差增量计算：kd * Δe（dt 并入 kd），
 * d_alpha 为 D 项一阶低通系数 [0, 1)，0 即不滤波 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float d_alpha;
    float sum_error_max;
    float output_max;
} GimbalPIDParam_t;

typedef struct {
    GimbalPIDParam_t angle;     ///< 角度环：关节角 deg -> deg/s
    GimbalPIDParam_t speed;     ///< 速度环：关节角速度 deg/s -> 电机指令
} GimbalAxisParam_t;

typedef struct {
    GimbalAxisParam_t axis[GIMBAL_AXIS_COUNT];
} GimbalGainSet_t;

typedef struct {
    PIDStruct angle_pid;
    PIDStruct speed_pid;
    float angle_d_alpha;        ///< D 项低通系数，随增益集加载
    float speed_d_alpha;
    float angle_out_dps;        ///< 角度环输出，分频周期间保持
    float speed_ref_dps;        ///< 角度环输出 + 关节角速度前馈
    float feedforward;          ///< 模型前馈（重力补偿等），暂为 0
    float output;               ///< 写入电机的最终指令
} GimbalAxisCtrl_t;

/* 逆运动学解出的关节空间指令 */
typedef struct {
    float angle_deg[GIMBAL_AXIS_COUNT];
    float omega_dps[GIMBAL_AXIS_COUNT];
} GimbalJointRef_t;

typedef struct {
    GimbalAxisCtrl_t axis[GIMBAL_AXIS_COUNT];
    GimbalJointRef_t joint_ref;
    RateDivider angle_loop;     ///< 角度环相对速度环的分频
} GimbalController_t;

/*--------------------------------------对象与构造参数--------------------------------------*/
/* 电机 inverse 同时决定指令和编码器反馈的方向：正指令应让关节角增大
 * 电机 zero_point 标定在枪管水平、朝底盘正前方的位置 */
typedef struct {
    GimbalModel_t model;
    Motor_t *yaw_motor;
    Motor_t *pitch_motor;
    GimbalGainSet_t normal_gain;
    GimbalGainSet_t sniper_gain;
    const INS_t *ins;           ///< 安装在 pitch 连杆上的 IMU，轴向与末端系对齐
    float pitch_min_deg;        ///< pitch 关节角限位
    float pitch_max_deg;
    uint16_t angle_loop_div;    ///< 每 N 次 Gimbal_Control 跑一次角度环，0/1 为不分频
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
    float pitch_min_deg;
    float pitch_max_deg;
} Gimbal_t;

void Gimbal_Init(Gimbal_t *gimbal, const GimbalConfig_t *config);
/* 输入层：状态切换，世界系目标折算
 * 保护状态下忽略外部角度，目标由控制层跟随当前姿态；
 * 上层做增量控制时应以 gimbal->target 为基准，pitch 超出关节限位时控制层会把目标改写为可达方向 */
void Gimbal_Input(Gimbal_t *gimbal, const GimbalInput_t *input);
/* 估计层：由 INS 和编码器更新关节角与三个旋转矩阵 */
void Gimbal_Estimate(Gimbal_t *gimbal);
/* 控制层：世界系目标 -> 底盘系 -> 关节角（逆运动学），再做关节空间双环 PID
 * 调用频率即速度环频率，角度环按 angle_loop_div 分频；
 * 角度环 PID 参数按分频后的周期整定 */
void Gimbal_Control(Gimbal_t *gimbal);

#endif

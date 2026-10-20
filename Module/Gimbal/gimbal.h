#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

#include "CAN_driver.h"
#include "FreeRTOS.h"
#include "Motor.h"
#include "algorism.h"
#include "dh.h"
#include "ins.h"
#include "pid.h"
#include "queue.h"

/* 角度统一 deg，角速度 deg/s
 * 每个控制周期调用 Gimbal_Run，内部依次：目标输入 -> 观测 -> 控制 -> 通信
 * 坐标系与关节角定义见 gimbal.md：底盘系 X 前、Y 左、Z 上，末端系 X 前、Y 右、Z 下
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
    GIMBAL_FRAME_END,       ///< {G}：末端云台系，X 前 Y 右 Z 下，与 IMU 对齐
    GIMBAL_FRAME_COUNT
} GimbalFrame_t;

/*--------------------------------------物理模型--------------------------------------*/
/* DH 表按 gimbal.md 填写（改进 DH），旋转部分须满足 R03 = Rz(yaw) * Ry(-pitch) * Rx(180°)：
 * yaw   | α=0   | a=0  | d=0 | θ=faiz
 * pitch | α=90  | a=-l | d=0 | θ=theta
 * end   | α=90  | a=L  | d=0 | θ=0
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

/* 上层（状态机、自瞄）经消息队列发给云台的命令，角速度为世界系 */
typedef struct {
    GimbalState_t state;
    float yaw_omega_dps;
    float pitch_omega_dps;
} GimbalCmd_t;

/* 队列里的元素：命令加发布时刻，云台据此判断上层是否失联 */
typedef struct {
    GimbalCmd_t cmd;
    TickType_t publish_tick;
} GimbalCmdMail_t;

/* 输入层处理后的世界系目标 */
typedef struct {
    float pitch_deg;
    float yaw_deg;
    float pitch_omega_dps; ///< 期望世界角速度，映射到关节后作为速度环前馈
    float yaw_omega_dps;
} GimbalTarget_t;

/*--------------------------------------估计层--------------------------------------*/
typedef struct {
    float pitch_deg;                            ///< 末端世界指向，INS 约定，由枪管 x 轴方向得到
    float yaw_deg;
    float joint_deg[GIMBAL_AXIS_COUNT];         ///< 编码器关节角，方向按 motor->inverse 折算
    float joint_omega_dps[GIMBAL_AXIS_COUNT];   ///< 编码器关节角速度（相对底盘）
    float gimbal_to_world[3][3];                ///< 末端 -> 世界，由 INS 四元数构造
    float gimbal_to_base[3][3];                 ///< 末端 -> 底盘，由 DH 正运动学构造
    float base_to_world[3][3];                  ///< 底盘 -> 世界，由前两者反解
    uint8_t valid;                             ///< INS 就绪且本周期取到数据
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
/* 初始化参数按依赖从底到顶分层：外设 -> 通信 -> 电机 -> 状态 -> 参数
 * 电机 inverse 只翻转指令方向，反馈由估计层按 Motor_Direction 折算：正指令应让关节角增大
 * 电机 zero_point 标定在枪管水平、朝底盘正前方的位置 */

/*--------------------------------------外设层--------------------------------------*/
typedef struct {
    CanInstance_t *yaw_can;     ///< yaw 电机所在 CAN，云台在初始化里把电机反馈绑上去
    CanInstance_t *pitch_can;
} GimbalPeriph_t;

/*--------------------------------------通信层--------------------------------------*/
typedef struct {
    QueueHandle_t cmd_queue;    ///< 元素 GimbalCmdMail_t，长度 1 覆盖式，由 System 层创建
    QueueHandle_t ins_queue;    ///< 元素 INSMsg_t，装在 pitch 连杆上的 IMU，轴向与末端系对齐
} GimbalComm_t;

/*--------------------------------------电机层--------------------------------------*/
typedef struct {
    Motor_t *yaw;
    Motor_t *pitch;
} GimbalMotor_t;

/*--------------------------------------状态层--------------------------------------*/
typedef struct {
    GimbalState_t init_state;
} GimbalStateInit_t;

/*--------------------------------------参数层--------------------------------------*/
typedef struct {
    GimbalModel_t model;
    GimbalGainSet_t normal_gain;
    GimbalGainSet_t sniper_gain;
    float pitch_min_deg;        ///< pitch 关节角限位
    float pitch_max_deg;
    uint16_t angle_loop_div;    ///< 每 N 次 Gimbal_Control 跑一次角度环，0/1 为不分频
    float period_s;             ///< Gimbal_Run 的调用周期
    uint32_t cmd_timeout_ms;    ///< 命令超过这个时间没更新就按失联进保护
} GimbalParam_t;

typedef struct {
    GimbalPeriph_t periph;
    GimbalComm_t comm;
    GimbalMotor_t motor;
    GimbalStateInit_t state;
    GimbalParam_t param;
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
    GimbalComm_t comm;
    float pitch_min_deg;
    float pitch_max_deg;
    float period_s;
    uint32_t cmd_timeout_ms;
    uint16_t yaw_enable_divider;    ///< yaw 使能重发计数
    uint8_t yaw_protect_slot;       ///< 保护态下失能、清错的补发节拍
} Gimbal_t;

void Gimbal_Init(Gimbal_t *gimbal, const GimbalConfig_t *config);
void Gimbal_Run(Gimbal_t *gimbal);
void Gimbal_Input(Gimbal_t *gimbal, const GimbalInput_t *input);
void Gimbal_Estimate(Gimbal_t *gimbal);
void Gimbal_Control(Gimbal_t *gimbal);

#endif

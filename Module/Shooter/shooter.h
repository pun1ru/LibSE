#ifndef SHOOTER_H
#define SHOOTER_H

#include <stdint.h>

#include "CAN_driver.h"
#include "FreeRTOS.h"
#include "Motor.h"
#include "pid.h"
#include "queue.h"

/* 摩擦轮模块，转速统一 rpm（电机转子转速），支持 1 ~ SHOOTER_WHEEL_MAX 个摩擦轮，数量由 motor.wheel_count 决定
 * 所有摩擦轮共用同一个目标转速和同一组速度环 PID，每个轮子各有一个 PID 实例
 * 每个控制周期调用 Shooter_Run，内部依次：目标输入 -> 观测 -> 控制 -> 通信
 * 电机 inverse 只翻转指令方向，反馈由观测层按 Motor_Direction 折算：正目标转速应让该轮沿发射方向转动
 * PID 输出直接写 motorcmd.current，按电机协议给定值计（C620 为 +-16384），output_max 要不超过电机上限
 */
#define SHOOTER_WHEEL_MAX   6U

/*--------------------------------------状态机--------------------------------------*/
typedef enum {
    SHOOTER_PROTECTED,  /* 保护：输出清零，PID 复位 */
    SHOOTER_STANDBY,    /* 待机：摩擦轮以 standby_rpm 低速转动 */
    SHOOTER_ENABLED     /* 使能：摩擦轮闭环到 param.fric_rpm */
} ShooterState_t;

/*--------------------------------------输入层--------------------------------------*/
/* 上层（状态机）经消息队列发给摩擦轮的命令，只给状态，转速是模块的固有参数 */
typedef struct {
    ShooterState_t state;
} ShooterCmd_t;

/* 队列里的元素：命令加发布时刻，摩擦轮据此判断上层是否失联 */
typedef struct {
    ShooterCmd_t cmd;
    TickType_t publish_tick;
} ShooterCmdMail_t;

/*--------------------------------------估计层--------------------------------------*/
typedef struct {
    float speed_rpm[SHOOTER_WHEEL_MAX];             /* 折算到发射方向的转速 */
    uint32_t feedback_stamp[SHOOTER_WHEEL_MAX];     /* 上次见到的电机反馈计数 */
    uint16_t stale_count[SHOOTER_WHEEL_MAX];        /* 反馈计数连续没变化的周期数，到 feedback_timeout_cycles 判失联 */
    uint8_t valid;                                  /* 所有摩擦轮的反馈都在更新 */
} ShooterObserver_t;

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
} ShooterPIDParam_t;

typedef struct {
    PIDStruct speed_pid[SHOOTER_WHEEL_MAX];         /* 速度环：转速 rpm -> 电机指令 */
    float output[SHOOTER_WHEEL_MAX];                /* 写入电机的最终指令 */
} ShooterController_t;

/*--------------------------------------对象与构造参数--------------------------------------*/
/* 初始化参数按依赖从底到顶分层：外设 -> 通信 -> 电机 -> 状态 -> 参数 */

/*--------------------------------------外设层--------------------------------------*/
typedef struct {
    CanInstance_t *can[SHOOTER_WHEEL_MAX];  /* 第 i 个摩擦轮所在的 CAN，模块在初始化里把电机反馈绑上去 */
} ShooterPeriph_t;

/*--------------------------------------通信层--------------------------------------*/
typedef struct {
    QueueHandle_t cmd_queue;    /* 元素 ShooterCmdMail_t，长度 1 覆盖式，由 System 层创建 */
} ShooterComm_t;

/*--------------------------------------电机层--------------------------------------*/
typedef struct {
    Motor_t *wheel[SHOOTER_WHEEL_MAX];      /* 只用前 wheel_count 个，需为电流模式的电机（如 RM C610 / C620）
                                             * 所有轮子必须在同一个电机组（同一帧），模块只经 wheel[0] 发一次组指令 */
    uint8_t wheel_count;
} ShooterMotor_t;

/*--------------------------------------状态层--------------------------------------*/
typedef struct {
    ShooterState_t init_state;
} ShooterStateInit_t;

/*--------------------------------------参数层--------------------------------------*/
typedef struct {
    ShooterPIDParam_t speed;
    float fric_rpm;                     /* 使能转速，沿发射方向为正 */
    float standby_rpm;                  /* 待机转速，有符号，沿发射方向为正 */
    uint32_t cmd_timeout_ms;            /* 命令超过这个时间没更新就按失联进保护 */
    uint16_t feedback_timeout_cycles;   /* 电机反馈连续这么多个周期没更新就按失联，输出清零 */
} ShooterParam_t;

typedef struct {
    ShooterPeriph_t periph;
    ShooterComm_t comm;
    ShooterMotor_t motor;
    ShooterStateInit_t state;
    ShooterParam_t param;
} ShooterConfig_t;

typedef struct {
    ShooterConfig_t config;
    ShooterState_t state;
    ShooterObserver_t observer;
    ShooterController_t controller;
} Shooter_t;

void Shooter_Init(Shooter_t *shooter, const ShooterConfig_t *config);
void Shooter_Run(Shooter_t *shooter);
/* 输入层：状态切换时复位 PID */
void Shooter_Input(Shooter_t *shooter, const ShooterCmd_t *cmd);
/* 估计层：折算转速，检查各轮反馈是否还在更新 */
void Shooter_Estimate(Shooter_t *shooter);
/* 控制层：保护或观测无效时清零，否则每个轮子一个速度环 */
void Shooter_Control(Shooter_t *shooter);

#endif

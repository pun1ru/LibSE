#ifndef STIR_H
#define STIR_H

#include <stdint.h>

#include "Motor.h"
#include "algorism.h"
#include "pid.h"

/* 拨盘模块，角度统一 deg，角速度 deg/s，目标与反馈都在连续（多圈）角度坐标下
 * 每个控制周期依次调用：Stir_Input -> Stir_Estimate -> Stir_Control
 * 拨盘一圈有 pocket_count 个弹位，一次拨弹转过一个弹位，沿角度减小方向（参考 hero_down）
 * 当前 Stir_Input 与堵转检测已实现，估计层与控制层待部署
 */

/*--------------------------------------状态机--------------------------------------*/
typedef enum {
    STIR_PROTECTED,  /* 保护：输出清零，目标跟随观测 */
    STIR_MOVING,     /* 移动：目标与反馈误差大于到位容差 */
    STIR_LOCATED,    /* 到位：误差在容差内，可以接受下一次拨弹 */
    STIR_BLOCKED     /* 堵转：低速高扭矩持续 */
} StirState_t;

/*--------------------------------------输入层--------------------------------------*/
/* 只有 STIR_PROTECTED 由上层指定，其余状态由模块根据反馈判定 */
typedef struct {
    StirState_t state;
    uint8_t fire_request;       /* 非 0 表示请求拨弹，释放后才能触发下一次 */
} StirInput_t;

/*--------------------------------------估计层--------------------------------------*/
typedef struct {
    float angle_deg;            /* 连续角度，方向按 motor->inverse 折算 */
    float omega_dps;
    float torque_nm;
    uint16_t block_count;       /* 堵转累计计数，低速高扭矩上升，其余衰减 */
    uint8_t blocked;            /* 计数到上限置 1，衰减到 0 才清除 */
    uint8_t valid;              /* 本周期取到有效反馈 */
} StirObserver_t;

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
} StirPIDParam_t;

typedef struct {
    StirPIDParam_t angle;    /* 角度环：角度 deg -> deg/s */
    StirPIDParam_t speed;    /* 速度环：角速度 deg/s -> 电机指令 */
} StirGain_t;

typedef struct {
    PIDStruct angle_pid;
    PIDStruct speed_pid;
    RateDivider angle_loop;     /* 角度环相对速度环的分频 */
    float angle_out_dps;        /* 角度环输出，分频周期间保持 */
    float output;               /* 写入电机的最终指令 */
} StirController_t;

/*--------------------------------------对象与构造参数--------------------------------------*/
/* 堵转判据：|omega| < speed_max_dps 且 |torque| > torque_min_nm 时计数每周期加 count_up，
 * 否则每周期减 count_down；计数限幅在 count_limit，到达上限判堵转 */
typedef struct {
    float speed_max_dps;
    float torque_min_nm;
    uint16_t count_up;
    uint16_t count_down;
    uint16_t count_limit;
} StirBlockParam_t;

/* 电机 inverse 同时决定指令和编码器反馈的方向：正指令应让拨盘角度增大 */
typedef struct {
    Motor_t *motor;
    StirGain_t gain;
    StirBlockParam_t block;
    uint8_t pocket_count;      /* 拨盘一圈的弹位数 */
    float arrive_deg;           /* MOVING -> LOCATED 的到位容差 */
    uint16_t angle_loop_div;    /* 每 N 次 Stir_Control 跑一次角度环，0/1 为不分频 */
    StirState_t init_state;
} StirConfig_t;

typedef struct {
    StirConfig_t config;
    StirState_t state;
    float target_deg;           /* 连续角度坐标下的拨盘目标 */
    uint8_t fire_hold;          /* 上一周期的拨弹请求 */
    uint16_t shot_count;        /* 累计拨弹次数 */
    StirObserver_t observer;
    StirController_t controller;
} Stir_t;

void Stir_Init(Stir_t *stir, const StirConfig_t *config);
/* 输入层：保护切换，拨弹请求转换为目标角度
 * 保护状态下忽略外部命令，目标由控制层跟随观测
 * 拨弹只在 LOCATED 且请求出现上升沿时触发一次，按住不放不会连发，未到位时的请求直接丢弃 */
void Stir_Input(Stir_t *stir, const StirInput_t *input);
/* 估计层：由电机反馈更新连续角度、角速度、扭矩与堵转计数 */
void Stir_Estimate(Stir_t *stir);
/* 控制层：按观测判定 MOVING / LOCATED / BLOCKED，再做角度环 + 速度环
 * 调用频率即速度环频率，角度环按 angle_loop_div 分频 */
void Stir_Control(Stir_t *stir);

#endif

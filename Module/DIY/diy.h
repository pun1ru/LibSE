#ifndef DIY_H
#define DIY_H

#include <stdint.h>

#include "Motor.h"
#include "dh.h"

/* 自定义控制器上的六轴小机械臂，PUMA560 构型，由操作者拖动，关节角用于遥操作
 * 角度统一 deg，长度统一 m
 * 关节命名沿用 engineer26orangediy：J1 Yaw、J2 PitchB、J3 PitchM、J4 RollB、J5 PitchF、J6 RollF
 * 电机 zero_point 标定在 DH 零位姿态，电机 inverse 使正指令对应关节角增大
 * 当前只定义对象与构造接口，输入、估计、控制层待部署
 */

/*--------------------------------------状态机--------------------------------------*/
typedef enum {
    DIY_PROTECTED,      /* 保护：电机失能，输出清零 */
    DIY_FREE            /* 自由拖动：零力矩加重力补偿，由操作者带动 */
} DIYState_t;

typedef enum {
    DIY_JOINT_YAW = 0,  /* J1：腰部 yaw */
    DIY_JOINT_PITCH_B,  /* J2：肩部 pitch */
    DIY_JOINT_PITCH_M,  /* J3：肘部 pitch */
    DIY_JOINT_ROLL_B,   /* J4：小臂 roll */
    DIY_JOINT_PITCH_F,  /* J5：腕部 pitch */
    DIY_JOINT_ROLL_F,   /* J6：腕部 roll */
    DIY_JOINT_COUNT
} DIYJoint_t;

/*--------------------------------------物理模型--------------------------------------*/
/* DH 表按改进 DH 填写，行 i 对应关节坐标系 {Ji}，列顺序与 DH_Param_t 一致：
 * J1 | α=0   | a=0  | d=0  | θ=q1
 * J2 | α=-90 | a=a1 | d=0  | θ=q2
 * J3 | α=0   | a=a2 | d=d3 | θ=q3
 * J4 | α=-90 | a=a3 | d=d4 | θ=q4
 * J5 | α=90  | a=0  | d=0  | θ=q5
 * J6 | α=-90 | a=0  | d=0  | θ=q6
 * 参考工程取值 a1=0、a2=218mm、a3=78mm、d3=0、d4=350mm，实际值由 Task 层按本机尺寸填写
 * 质量参数属于六个连杆，连杆 i 的质心表示在关节坐标系 {Ji} 下 */
typedef struct {
    DH_Param_t dh[DIY_JOINT_COUNT];
    float mass_kg[DIY_JOINT_COUNT];
    float com_m[DIY_JOINT_COUNT][3];            /* 质心在关节坐标系下的位置 [x, y, z] */
} DIYModel_t;

/*--------------------------------------电机与关节--------------------------------------*/
/*，模块只持有基类指针，由 Task 层创建并注册反馈回调 */
typedef struct {
    Motor_t *motor;
    float transmission_ratio;   /* 电机输出轴转角 / 关节转角，电机减速箱之外的传动，直驱为 1 */
    float min_deg;              /* 关节角限位 */
    float max_deg;
} DIYJointParam_t;

/*--------------------------------------对象与构造参数--------------------------------------*/
typedef struct {
    DIYModel_t model;
    DIYJointParam_t joint[DIY_JOINT_COUNT];
    DIYState_t init_state;
} DIYConfig_t;

typedef struct {
    DIYState_t state;
    DIYModel_t model;
    DIYJointParam_t joint[DIY_JOINT_COUNT];
} DIY_t;

void DIY_Init(DIY_t *diy, const DIYConfig_t *config);

#endif

#ifndef ARM_H
#define ARM_H

#include <stdbool.h>
#include <stdint.h>

#include "CAN_driver.h"
#include "Motor.h"
#include "dh.h"
#include "pid.h"

/* 七轴臂模块，对象是 J0 ~ J6 七个关节电机，角度统一 deg，位置统一 m
 * 每个控制周期调用 Arm_Run，内部依次：观测 -> 控制 -> 通信
 * 电机在 App 层的 motor_init 里创建，反馈帧在 Arm_Init 里绑到各自的 CAN 实例
 * 模块只定义类型和算法，DH 表、零点、角色这些具体值全部由 System 层的 module_init 在配置里给出
 * 主从角色在初始化前由配置定下，之后不再修改：主机状态 PROTECTED/ENABLE/FEEDBACK/MASTER，从机状态 PROTECTED/ENABLE/AUTO/SERVANT
 */
#define ARM_JOINT_COUNT   7U
#define ARM_VECTOR_DIM    3U    /* 位置、欧拉角的分量数 */

/*--------------------------------------状态机--------------------------------------*/
typedef enum {
    ARM_ROLE_MASTER = 0,    /* 主机：读操作者输入，向从机发布关节目标 */
    ARM_ROLE_SERVANT        /* 从机：接收主机目标，驱动关节 */
} ArmRole_t;

typedef enum {
    ARM_PROTECTED = 0,  /* 保护：每个电机发零指令，主从共有 */
    ARM_ENABLE,         /* 使能：电机使能并跑关节角闭环，目标默认对齐当前关节角，主从共有 */
    ARM_FEEDBACK,       /* 主机：只读反馈，关节自由 */
    ARM_MASTER,         /* 主机：主机正常工作 */
    ARM_AUTO,           /* 从机：从机自主运行 */
    ARM_SERVANT         /* 从机：跟随主机 */
} ArmState_t;

typedef enum {
    ARM_J0 = 0,
    ARM_J1,
    ARM_J2,
    ARM_J3,
    ARM_J4,
    ARM_J5,
    ARM_J6
} ArmJoint_t;

/*--------------------------------------对象与构造参数--------------------------------------*/
/* 初始化参数按依赖从底到顶分层：外设 -> 电机 -> 状态 -> 参数，没有的层不写 */

/*--------------------------------------外设层--------------------------------------*/
typedef struct {
    CanInstance_t *can[ARM_JOINT_COUNT];    /* 第 i 个关节所在的 CAN，模块在初始化里把电机反馈绑上去 */
} ArmPeriph_t;

/*--------------------------------------电机层--------------------------------------*/
typedef struct {
    Motor_t *joint[ARM_JOINT_COUNT];        /* 下标即 ArmJoint_t，同一个 RM 电机组的电机不能重复发组指令 */
} ArmMotor_t;

/*--------------------------------------状态层--------------------------------------*/
typedef struct {
    ArmRole_t role;             /* 初始化前定下，之后不得修改 */
    ArmState_t init_state;      /* 必须属于 role 的状态集 */
} ArmStateInit_t;

/*--------------------------------------参数层--------------------------------------*/
/* 关节工作区间，观测角口径 deg，即 (-180, 180]，min 必须小于 max
 * 区间必须连续且不跨 ±180（量程超过 360 或跨 ±180 时挪零点，别把它拆成两段）
 * 只在非保护模式生效：目标进入区间内才允许，超出就截断到边界，保护模式不读它 */
typedef struct {
    float min_deg;
    float max_deg;
} ArmJointLimit_t;

/* 关节角度环增益：误差 deg -> 力矩 N*m，D 项按每周期误差增量计算（周期并入 kd），
 * d_alpha 为 D 项一阶低通系数 [0, 1)，0 即不滤波
 * output_max 为力矩限幅，0 会让输出恒为 0，填增益时务必一起填 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float d_alpha;
    float sum_error_max;
    float output_max;
} ArmJointGain_t;

/* DH 表为改进 DH，每个关节一行，第 i 行的 q 对应第 i 个关节，末端坐标系即 {J6}
 * zero_point 是各关节机械零位对应的电机原始编码器值，Arm_Init 写入电机，覆盖电机初始化时的默认值
 * direction 是关节 +q 方向相对编码器计数方向的符号：+1 表示正向转编码器增大，-1 表示减小
 * 它与 Motor_t.inverse 是两件事：direction 只管观测角的正负，inverse 只管力矩指令的正负 */
typedef struct {
    DH_Param_t dh[ARM_JOINT_COUNT];
    uint16_t zero_point[ARM_JOINT_COUNT];
    int8_t direction[ARM_JOINT_COUNT];
    ArmJointLimit_t limit[ARM_JOINT_COUNT];
    ArmJointGain_t gain[ARM_JOINT_COUNT];
} ArmParam_t;

typedef struct {
    ArmPeriph_t periph;
    ArmMotor_t motor;
    ArmStateInit_t state;
    ArmParam_t param;
} ArmConfig_t;

/*--------------------------------------观测--------------------------------------*/
typedef struct {
    float joint_deg[ARM_JOINT_COUNT];                   /* 关节角，方向已按 ArmParam_t.direction 折算 */
    float end_to_base[4][4];                            /* 末端到基座的齐次变换，DH 正运动学连乘 */
    float position_m[ARM_VECTOR_DIM];                   /* 末端在基座系下的位置 [x, y, z] */
    float euler_deg[ARM_VECTOR_DIM];                    /* 末端姿态 [roll, pitch, yaw]，Z-Y-X 顺序，下标见 EULER_ROLL 等 */
} ArmObserver_t;

/*--------------------------------------控制层--------------------------------------*/
/* 每关节一个角度环。误差直接用目标减观测，不做最短路径折叠：
 * 限位区间连续且整段落在 (-180, 180] 内、不跨 ±180，直接相减就是区间内的那条路；
 * 折叠会让跨度超过 180 的关节走 ±180 那侧的禁行区 */
typedef struct {
    PIDStruct pid;          /* 角度环 PID 状态 */
    float target_deg;       /* 本周期目标角，已按工作区间截断 */
    float output;           /* 角度环输出，力矩口径 N*m */
} ArmJointCtrl_t;

typedef struct {
    ArmConfig_t config;
    ArmState_t state;
    ArmObserver_t observer;
    ArmJointCtrl_t joint_ctrl[ARM_JOINT_COUNT];
    uint16_t enable_divider;    /* 使能帧重发倒计时，0 表示本周期发 */
    uint8_t ctrl_align_pending; /* 进入非保护态后第一次控制前，把目标对齐观测并清 PID */
} Arm_t;

void Arm_Init(Arm_t *arm, const ArmConfig_t *config);
void Arm_Run(Arm_t *arm);
/* 状态由上层（遥控输入处理）经此修改，目标状态不属于本机角色时拒绝并返回 false */
bool Arm_SetState(Arm_t *arm, ArmState_t state);

#endif

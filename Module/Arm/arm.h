#ifndef ARM_H
#define ARM_H

#include <stdbool.h>
#include <stdint.h>

#include "CAN_driver.h"
#include "Motor.h"
#include "arm_dynamics.h"
#include "dh.h"
#include "ik_analytic.h"
#include "ik_numeric.h"
#include "pid.h"

/* 七轴臂模块，对象是 J0 ~ J6 七个关节电机，角度统一 deg，位置统一 m
 * 每个控制周期调用 Arm_Run，内部依次：观测 -> 控制 -> 通信
 * 电机在 App 层的 motor_init 里创建，反馈帧在 Arm_Init 里绑到各自的 CAN 实例
 * 模块只定义类型和算法，DH 表、零点、角色这些具体值全部由 System 层的 module_init 在配置里给出
 * 主从角色在初始化前由配置定下，之后不再修改；IK/TEST 为两种角色都可用的遥控测试状态。
 */
#define ARM_JOINT_COUNT   7U
#define ARM_VECTOR_DIM    3U    /* 位置、欧拉角的分量数 */

/* 重力前馈是否叠加到力矩输出的编译开关：
 *   0（默认）只计算重力矩并存在 arm->gravity 里供观察，输出路径与没有这个功能时逐字节一致
 *   1 才会在 Arm_JointControl 里把 gravity.torque_cmd 加到控制输出上
 * 实机采集重力辨识数据时必须保持 0 或把 gravity.enable 置 0，否则采到的力矩里已经含了前馈 */
#ifndef ARM_GRAVITY_OUTPUT
#define ARM_GRAVITY_OUTPUT  1
#endif

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
    ARM_SERVANT,        /* 从机：跟随主机 */
    ARM_HOST,           /* 主从共有：上位机设定角度目标，是否闭环由上位机决定 */
    ARM_IK,             /* 主从共有：遥控器输入 TCP 六维位姿，数值 IK 更新关节目标 */
    ARM_TEST = 8        /* 主从共有：遥控器关节角度测试 */
} ArmState_t;

/* ARM_TEST 下 CH1 的增量步长按 2 ms 控制周期换算，满杆约 10 deg/s。
 * CH4 使用回差边沿，拨过阈值只切一次，回中后才允许再次切换。 */
#define ARM_TEST_CH1_DEADBAND             20
#define ARM_TEST_TARGET_MAX_STEP_DEG      0.02f
#define ARM_TEST_CH4_SWITCH_THRESHOLD     500
#define ARM_TEST_CH4_RELEASE_THRESHOLD    300

/* ARM_IK 遥控器输入：满杆速度 0.05 m/s、5 deg/s，按 2 ms 周期折算为单周期步长。
 * 定位模式 CH1=x、CH2=y、CH3=z；姿态模式 CH0=roll、CH3=pitch、CH2=yaw。 */
#define ARM_IK_RC_POS_STEP_M              0.0001f
#define ARM_IK_RC_ROT_STEP_DEG            0.01f
#define ARM_IK_RC_PITCH_LIMIT_DEG         89.0f

typedef enum {
    ARM_HOST_TARGET_OK = 0,
    ARM_HOST_TARGET_WRONG_STATE,
    ARM_HOST_TARGET_BAD_TARGET
} ArmHostTargetResult_t;

typedef enum {
    ARM_IK_TYPE_NUMERIC = 0,
    ARM_IK_TYPE_ANALYTIC
} ArmIkType_t;

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
/* 关节工作区间，单圈绝对角口径 deg，即 (-180, 180]，min 必须小于 max
 * 区间必须连续且不跨 ±180（量程超过 360 或跨 ±180 时挪零点，别把它拆成两段）
 * 多圈控制时映射到当前圈：limit_multi = current_multi + (limit_abs - current_abs)
 * 差值不做环绕，保证限位和当前实际角在同一圈 */
typedef struct {
    float min_deg;
    float max_deg;
} ArmJointLimit_t;

/* 关节角度环增益：误差 deg -> 力矩 N*m，D 项按每周期误差增量计算（周期并入 kd），
 * d_alpha 为 D 项一阶低通系数 [0, 1)，0 即不滤波
 * output_max 只限制 PID 自身输出；total_output_max 限制 PID + 重力前馈的最终总和
 * 两者都应填正数，未填或非正时对应部分输出为 0 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float d_alpha;
    float sum_error_max;
    float output_max;
    float total_output_max;
} ArmJointGain_t;

/* 数值逆解参数：numeric 的含义见 ik_numeric.h；weight 是关节权重（越大越不愿动，Arm_SetNumericIkTarget 传 NULL 时用它）
 * 没填（numeric 全 0）时数值逆解被拒绝，与增益没填时关节不动的约定一致 */
typedef struct {
    IkNumericParam_t numeric;
    float weight[ARM_JOINT_COUNT];
} ArmIkParam_t;

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
    ArmGravityParam_t gravity;      /* 重力补偿：开关、符号、g、参数表指针，见 arm_dynamics.h */
    ArmIkParam_t ik;                /* 数值逆解 */
} ArmParam_t;

typedef struct {
    ArmPeriph_t periph;
    ArmMotor_t motor;
    ArmStateInit_t state;
    ArmParam_t param;
} ArmConfig_t;

/*--------------------------------------观测--------------------------------------*/
typedef struct {
    float joint_deg[ARM_JOINT_COUNT];                   /* 多圈关节角，方向已按 ArmParam_t.direction 折算 */
    float joint_abs_deg[ARM_JOINT_COUNT];               /* 单圈绝对角，用于把绝对限位映射到当前圈 */
    float end_to_base[4][4];                            /* DH 末端到基座的齐次变换，仅供底层 IK 换算 */
    float tcp_to_base[4][4];                            /* TCP 到基座的齐次变换：DH 末端沿 z 偏移 -0.045 m */
    float position_m[ARM_VECTOR_DIM];                   /* TCP 在基座系下的位置 [x, y, z] */
    float euler_deg[ARM_VECTOR_DIM];                    /* TCP 姿态 [roll, pitch, yaw]，Z-Y-X 顺序 */
    float tcp_position_m[ARM_VECTOR_DIM];               /* TCP 在基座系下的位置 [x, y, z] */
    float tcp_euler_deg[ARM_VECTOR_DIM];                /* TCP 姿态 [roll, pitch, yaw]，Z-Y-X 顺序 */
} ArmObserver_t;

/*--------------------------------------控制层--------------------------------------*/
/* 每关节一个角度环。误差直接用目标减观测，不做最短路径折叠：
 * 限位区间连续且整段落在 (-180, 180] 内、不跨 ±180，直接相减就是区间内的那条路；
 * 折叠会让跨度超过 180 的关节走 ±180 那侧的禁行区 */
typedef struct {
    PIDStruct pid;          /* 角度环 PID 状态 */
    float target_deg;       /* 多圈目标角，使能时持续跟踪观测并限制在入场锁存区间 */
    float limit_min_deg;    /* 进使能时锁存的多圈下限 */
    float limit_max_deg;    /* 进使能时锁存的多圈上限 */
    float output;           /* 角度环输出，力矩口径 N*m */
} ArmJointCtrl_t;

/* 重力补偿运行状态，每个 Arm_Run 在正运动学之后刷新
 * model_valid 为 0 时（未使能 / 无参数表 / 关节角含 NaN）torque_model 与 torque_cmd 一律为 0 */
typedef struct {
    float torque_model[ARM_JOINT_COUNT];    /* 模型原始输出 dU/dq，N*m，不带符号、不限幅，供观察和对拍 */
    float torque_cmd[ARM_JOINT_COUNT];      /* sign 之后的重力前馈，不做单轴预限幅；与 PID 相加后统一受 total_output_max 限制 */
    uint8_t model_valid;
    uint8_t output_active;                  /* 本周期是否真的把 torque_cmd 加进了输出，0 = 只计算没输出 */
} ArmGravityState_t;

/* 逆解运行状态。NUMERIC 每周期推进一个微分步；ANALYTIC 在设置目标时一次求出全部关节角。
 * 单关节目标、切换状态、停止 HOST 控制都会把 active 清零。 */
typedef struct {
    uint8_t active;
    uint8_t limit_mask;             /* bit i = 上一步里关节 i 被限位钳住 */
    uint8_t step_limited;           /* 上一步的步长约束起了作用 */
    uint8_t converged;              /* 上一步前误差已低于收敛阈值 */
    uint8_t stalled;                /* 没到位却几乎不动达到 ARM_IK_STALL_CYCLES：够不到，或被限位 / 奇异挡住 */
    uint16_t stall_cycles;
    float pos_err_m;                /* 上一步的位置误差 */
    float rot_err_deg;              /* 上一步的姿态误差 */
    float target[4][4];             /* 末端目标位姿，基座系 */
    float weight[ARM_JOINT_COUNT];
    float reference_deg[ARM_JOINT_COUNT];   /* 零空间参考构型，进入数值逆解时锁定 */
    IkAnalyticBranch_t analytic_branch;
    float analytic_psi_deg;
    uint8_t analytic_valid;
    uint8_t analytic_shoulder_singular;
    uint8_t analytic_wrist_singular;
} ArmIkState_t;

/* 使能与 HOST 控制状态 */
typedef struct {
    uint8_t ctrl_align_pending; /* 进入非保护态后第一次控制前，把目标对齐观测并清 PID */
    uint8_t host_align_pending; /* 进入 HOST 后第一次控制前，锁存限位并把目标对齐观测 */
    uint8_t host_closed_loop_mask;  /* bit i=1：关节 i 追 HOST 目标；0 位输出零力矩 */
    uint8_t host_target_valid_mask; /* bit i=1：关节 i 至少收到过一次 HOST 目标 */
    uint8_t rc_online;          /* RC frame is fresh; reported to the host status frame */
    float host_target_deg[ARM_JOINT_COUNT]; /* 上位机给出的多圈目标，用于状态显示/复查 */
} ArmHost_t;

typedef struct {
    uint8_t align_pending;          /* First ARM_TEST cycle aligns targets to feedback. */
    uint8_t selected_joint;         /* Joint controlled by CH1. */
    uint8_t ch4_switch_armed;       /* Re-armed after CH4 returns below release threshold. */
    int16_t ch1;
    int16_t ch4;
} ArmTest_t;

typedef struct {
    uint8_t align_pending;          /* 首次进入 ARM_IK 时用当前 TCP 位姿建立目标 */
    uint8_t attitude_mode;          /* 0：位置输入；1：姿态输入 */
    uint8_t ch4_switch_armed;       /* CH4 回中后允许下一次切换 */
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    float target_position_m[ARM_VECTOR_DIM];
    float target_euler_deg[ARM_VECTOR_DIM];
} ArmIkRc_t;

typedef struct {
    ArmConfig_t config;
    ArmState_t state;
    ArmIkType_t ik_type;
    ArmObserver_t observer;
    ArmJointCtrl_t joint_ctrl[ARM_JOINT_COUNT];
    ArmGravityState_t gravity;
    ArmHost_t host;
    ArmTest_t test;
    ArmIkRc_t ik_rc;
    ArmIkState_t ik;
} Arm_t;

void Arm_Init(Arm_t *arm, const ArmConfig_t *config);
void Arm_Run(Arm_t *arm);
/* 状态由上层（遥控输入处理）经此修改，目标状态不属于本机角色时拒绝并返回 false */
bool Arm_SetState(Arm_t *arm, ArmState_t state);
/* Cache the receiver freshness for protocol status reporting. */
void Arm_SetRcOnline(Arm_t *arm, bool online);
/* Cache ARM_TEST RC inputs before Arm_Run; target update only consumes these values. */
void Arm_SetTestInput(Arm_t *arm, int16_t ch1, int16_t ch4);
/* Cache all DT7 channels once per control period for ARM_TEST and ARM_IK. */
void Arm_SetRcInput(Arm_t *arm, int16_t ch0, int16_t ch1, int16_t ch2,
                    int16_t ch3, int16_t ch4);

/* HOST 单关节目标：joint 为 0..ARM_JOINT_COUNT-1。multi_turn=true 时 target_deg 为多圈角，
 * 必须落在进入 HOST 时锁存的多圈限位内；false 时按旧协议解释为单圈绝对角并映射到当前圈。
 * 只更新指定关节，其他关节保持不变。
 * closed_loop=false 时保存目标，该关节不跑 PID（只剩重力前馈；前馈关闭时输出零力矩）；只有 HOST 状态接受 */
ArmHostTargetResult_t Arm_SetHostTarget(Arm_t *arm, uint8_t joint, float target_deg,
                                        bool closed_loop, bool multi_turn);
/* HOST 链路失效或退出目标控制：关闭全部闭环、清 PID 与上一周期输出
 * 注意：状态仍是 ARM_HOST，下个 Arm_Run 起重力前馈照常叠加（遵从 ENABLE 的逻辑），
 * 所以这不再等于“零力矩”：重力前馈打开时关节会被前馈托住；要真正泄力需切 ARM_PROTECTED 或关掉重力前馈 */
void Arm_StopHostControl(Arm_t *arm);

/* 重力补偿，Arm_Run 在正运动学之后每周期调用，其他地方一般不用直接调
 * 计算原始重力矩（gravity.torque_model），再乘 param.gravity.sign 得到独立的重力指令 torque_cmd；
 * 与 PID 相加后由 Arm_JointControl 按各关节 gain.total_output_max 统一限幅
 * 参数表为空 / 未使能 / 关节角非有限值时全部清零，不会残留上一周期的值 */
void Arm_GravityUpdate(Arm_t *arm);
/* 把 torque_cmd 清零，退出控制或关闭补偿时用 */
void Arm_GravityClear(Arm_t *arm);

#endif

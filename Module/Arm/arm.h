#ifndef ARM_H
#define ARM_H

#include <stdint.h>

#include "CAN_driver.h"
#include "Motor.h"

/* 七轴臂模块，对象是 J0 ~ J6 七个关节电机
 * 每个控制周期调用 Arm_Run，目前只有保护状态：每个电机发一帧各自控制模式下的零指令
 * 电机在 App 层的 motor_init 里创建，反馈帧在 Arm_Init 里绑到各自的 CAN 实例
 * 零指令保持电机初始化时的控制模式，只把指令量清零：力矩模式发零力矩，MIT 模式发零位置零速度零增益零力矩
 */
#define ARM_JOINT_COUNT   7U

/*--------------------------------------状态机--------------------------------------*/
typedef enum {
    ARM_PROTECTED   /* 保护：每个电机发零指令 */
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
/* 初始化参数按依赖从底到顶分层：外设 -> 电机 -> 状态，没有的层不写 */

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
    ArmState_t init_state;
} ArmStateInit_t;

typedef struct {
    ArmPeriph_t periph;
    ArmMotor_t motor;
    ArmStateInit_t state;
} ArmConfig_t;

typedef struct {
    ArmConfig_t config;
    ArmState_t state;
} Arm_t;

void Arm_Init(Arm_t *arm, const ArmConfig_t *config);
void Arm_Run(Arm_t *arm);
/* 每个电机发一帧各自控制模式下的零指令，不改电机的控制模式
 * 经 sendcommand 逐个发：J0、J2 在不同的 RM 电机组（不同发送帧），不会重复发同一帧 */
void Arm_SendZeroCommand(Arm_t *arm);

#endif

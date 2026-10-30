#ifndef ARM_JOINTCTRL_H
#define ARM_JOINTCTRL_H

#include "arm.h"

/*--------------------------------------控制--------------------------------------*/
/* 进使能态的第一次控制前调用：目标取当前多圈关节角，同时清 PID 和输出 */
void Arm_AlignJointTarget(Arm_t *arm);
/* 非使能态每周期调用：目标跟随当前多圈关节角 */
void Arm_TrackJointTarget(Arm_t *arm);
/* 关节角度环：目标持续跟踪当前多圈观测，仅在观测离开进使能时锁存的限位时截断到边界
 * 限位在退保护进使能时按当前圈映射一次；误差不做最短路径折叠，输出力矩写进各电机并发送 */
void Arm_JointControl(Arm_t *arm);

/*--------------------------------------通信--------------------------------------*/
/* 每个电机发一帧各自控制模式下的零指令，不改电机的控制模式
 * 经 sendcommand 逐个发：J0、J2 在不同的 RM 电机组（不同发送帧），不会重复发同一帧 */
void Arm_SendZeroCommand(Arm_t *arm);
/* 把 joint_ctrl 里的输出按各电机控制模式写成指令并发送，先全部写完再统一发 */
void Arm_SendJointCommand(Arm_t *arm);
/* 给每个提供使能操作的电机发一帧使能，RM 电机没有使能帧，跳过 */
void Arm_SendEnableCommand(Arm_t *arm);
/* 给每个提供失能操作的电机发一帧失能，没有失能实现的电机跳过 */
void Arm_SendDisableCommand(Arm_t *arm);

#endif

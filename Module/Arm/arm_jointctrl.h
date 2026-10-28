#ifndef ARM_JOINTCTRL_H
#define ARM_JOINTCTRL_H

#include "arm.h"

/*--------------------------------------控制--------------------------------------*/
/* 进使能态的第一次控制前调用：目标取当前关节角并截断到工作区间，同时清 PID 和输出 */
void Arm_AlignJointTarget(Arm_t *arm);
/* 关节角度环：目标减观测直接进 PID（不做最短路径折叠，原因见 .c），输出力矩写进各电机并发送
 * 目标每周期按工作区间截断；限位只在非保护模式生效 */
void Arm_JointControl(Arm_t *arm);

/*--------------------------------------通信--------------------------------------*/
/* 每个电机发一帧各自控制模式下的零指令，不改电机的控制模式
 * 经 sendcommand 逐个发：J0、J2 在不同的 RM 电机组（不同发送帧），不会重复发同一帧 */
void Arm_SendZeroCommand(Arm_t *arm);
/* 把 joint_ctrl 里的输出按各电机控制模式写成指令并发送，先全部写完再统一发 */
void Arm_SendJointCommand(Arm_t *arm);
/* 给每个提供使能操作的电机发一帧使能，RM 电机没有使能帧，跳过 */
void Arm_SendEnableCommand(Arm_t *arm);

#endif

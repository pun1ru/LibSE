#ifndef ARM_JOINTCTRL_H
#define ARM_JOINTCTRL_H

#include "arm.h"

/*--------------------------------------控制--------------------------------------*/
/* 进使能态的第一次控制前调用：目标取当前多圈关节角，同时清 PID 和输出 */
void Arm_AlignJointTarget(Arm_t *arm);
/* 进 HOST 的第一次控制前调用：锁存当前圈限位，目标对齐当前观测，PID 清零，默认不开闭环 */
void Arm_AlignHostTarget(Arm_t *arm);
/* 非使能态每周期调用：目标跟随当前多圈关节角 */
void Arm_TrackJointTarget(Arm_t *arm);
/* 按当前状态选择目标来源：ENABLE 跟踪当前观测，HOST 使用上位机目标，其他状态跟踪当前观测
 * ENABLE/HOST 的首次进入对齐、PID 清零也在这里处理 */
void Arm_UpdateTarget(Arm_t *arm);
/* 统一关节控制：内部按状态选择闭环关节，不再区分普通控制和 HOST 控制
 * ENABLE 全部关节闭环，HOST 由 host_closed_loop_mask 决定，其他状态发零指令
 * 限位在上一层的目标更新里处理；误差不做最短路径折叠，控制输出写进各电机并发送 */
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

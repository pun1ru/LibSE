#ifndef ARM_JOINTCTRL_H
#define ARM_JOINTCTRL_H

#include "arm.h"

/* 每个电机发一帧各自控制模式下的零指令，不改电机的控制模式
 * 经 sendcommand 逐个发：J0、J2 在不同的 RM 电机组（不同发送帧），不会重复发同一帧 */
void Arm_SendZeroCommand(Arm_t *arm);
/* 给每个提供使能操作的电机发一帧使能，RM 电机没有使能帧，跳过 */
void Arm_SendEnableCommand(Arm_t *arm);

#endif

#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include "arm.h"

/* 零点标定：把当前姿态记为机械零位，各电机的 zero_point 取当前原始编码器值
 * 任一关节还没收到反馈则不修改任何关节并返回 false；标定值只在 RAM 里，要保留需抄回 module_init 的配置 */
bool Arm_CalibrateZero(Arm_t *arm);
/* 电机反馈 -> 关节角，方向按 Motor_Direction 折算成与指令同向 */
void Arm_UpdateJointAngle(Arm_t *arm);
/* 关节角 -> 末端到基座的齐次变换，DH 正运动学 */
void Arm_ForwardKinematics(Arm_t *arm);
/* 末端到基座的变换 -> 三维坐标与欧拉角 */
void Arm_ExtractPose(Arm_t *arm);

#endif

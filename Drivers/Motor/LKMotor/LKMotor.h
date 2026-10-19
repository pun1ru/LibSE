#ifndef LKMOTOR_H
#define LKMOTOR_H

#include <stdint.h>

#include "CAN_driver.h"
#include "Motor.h"

#define LK_CAN_COMMAND_ID_BASE     0x140U
#define LK_CAN_FEEDBACK_ID_BASE    0x180U
#define LK_ENCODER_COUNTS          65536U
#define LK_IQ_COMMAND_LIMIT        2048.0f
#define LK_MF_CURRENT_PER_LSB      (33.0f / 4096.0f)
#define LK_MG_CURRENT_PER_LSB      (66.0f / 4096.0f)
#define LK_ANGLE_DEGREE_PER_LSB    0.01f
#define LK_SPEED_DPS_PER_LSB       1.0f
#define LK_SPEED_COMMAND_SCALE     100.0f

/* 扭矩常数取自选型册 LK_MOTOR.pdf 各型号规格表，单位 N*m/A，直接使用不再乘减速比 */
/* NM_PER_LSB = 扭矩常数 * 电流 LSB，即 iq 增加 1 对应的力矩 */
#define LK_MG5010E_I10_GEAR_RATIO         10.0f
#define LK_MG5010E_I10_TORQUE_CONSTANT    0.1f
#define LK_MG5010E_I10_NM_PER_LSB         (LK_MG5010E_I10_TORQUE_CONSTANT * LK_MG_CURRENT_PER_LSB)

/* MF 系列直驱，无减速箱 */
#define LK_MF4015_GEAR_RATIO              1.0f
#define LK_MF4015_TORQUE_CONSTANT         0.11f
#define LK_MF4015_NM_PER_LSB              (LK_MF4015_TORQUE_CONSTANT * LK_MF_CURRENT_PER_LSB)

/* MF4005 有两种绕组，规格表里 25T 和 40T 各一列 */
#define LK_MF4005_GEAR_RATIO              1.0f
#define LK_MF4005_25T_TORQUE_CONSTANT     0.05f
#define LK_MF4005_25T_NM_PER_LSB          (LK_MF4005_25T_TORQUE_CONSTANT * LK_MF_CURRENT_PER_LSB)
#define LK_MF4005_40T_TORQUE_CONSTANT     0.07f
#define LK_MF4005_40T_NM_PER_LSB          (LK_MF4005_40T_TORQUE_CONSTANT * LK_MF_CURRENT_PER_LSB)

#define LK_MG6012E_I8_GEAR_RATIO         8.0f
#define LK_MG6012E_I8_TORQUE_CONSTANT     1.09f
#define LK_MG6012E_I8_NM_PER_LSB          (LK_MG6012E_I8_TORQUE_CONSTANT * LK_MG_CURRENT_PER_LSB)

/* 具体型号，参数取自 LK_MOTOR.pdf 选型册和 CAN 协议；新增型号时在 LKMotor.c 的参数表里补一行 */
typedef enum {
    LK_MG6012E_I8,  ///< 双磁编，MG6012E-i8B（带刹车版）电气参数相同
    LK_MG5010E_I10, /* 减速比 10，参考臂 PitchB */
    LK_MF4015,      /* 直驱，参考臂 RollB、PitchF */
    LK_MF4005_25T,  /* 直驱，参考臂 RollF，绕组待确认 */
    LK_MF4005_40T,  /* 直驱，参考臂 RollF，绕组待确认 */
    LK_MODEL_COUNT
} LKMotorModel_t;

typedef struct {
    LKMotorModel_t model;
    uint8_t motor_id;
    uint16_t feedback_id;
    uint16_t zero_point;
    bool inverse;
} LKMotorConfig_t;

typedef struct {
    Motor_t base;
    LKMotorConfig_t motor_config;
} LKMotor_t;

void LKMotor_Init(LKMotor_t *motor, FDCAN_HandleTypeDef *can,
                  const LKMotorConfig_t *config);
void LKMotor_Feedback(LKMotor_t *motor, const uint8_t data[8], uint16_t rec_id);
void LKMotor_SendCommand(LKMotor_t *motor);
void LKMotor_RequestFeedback(LKMotor_t *motor);

#endif

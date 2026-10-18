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

/* 具体型号，参数取自 LK_MOTOR.pdf 选型册和 CAN 协议；新增型号时在 LKMotor.c 的参数表里补一行 */
typedef enum {
    LK_MG6012E_I8,  ///< 双磁编，MG6012E-i8B（带刹车版）电气参数相同
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

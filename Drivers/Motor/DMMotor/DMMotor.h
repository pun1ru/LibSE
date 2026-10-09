#ifndef DMMOTOR_H
#define DMMOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "CAN_driver.h"
#include "Motor.h"

typedef struct {
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float t_min;
    float t_max;
} LSB;

typedef enum{
    DM_4310,
    DM_4340,
    DM_6006,
    DM_8009
}MotorType_t;
typedef struct {
    MotorType_t motor_type;
    FDCAN_HandleTypeDef *can;
    uint16_t motor_id;
    uint16_t feedback_id;
    LSB lsb;
} MotorConfig_t;

typedef struct {
    Motor_t base;
    MotorConfig_t motor_config;
} DMMotor_t;

void DMMotor_Init(DMMotor_t *motor, const MotorConfig_t *config);
void DMMotor_SendCommand(DMMotor_t *motor);
void DMMotor_Feedback(DMMotor_t *motor, const uint8_t data[8], uint16_t rec_id);

#define DM4310_P_MIN -3.14f
#define DM4310_P_MAX 3.14f
#define DM4310_V_MIN -30.0f
#define DM4310_V_MAX 30.0f
#define DM4310_KP_MIN 0.0f
#define DM4310_KP_MAX 500.0f
#define DM4310_KD_MIN 0.0f
#define DM4310_KD_MAX 5.0f
#define DM4310_T_MIN -11.0f
#define DM4310_T_MAX 11.0f

#endif

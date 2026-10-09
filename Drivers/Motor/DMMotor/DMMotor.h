#ifndef DMMOTOR_H
#define DMMOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "CAN_driver.h"
#include "Motor.h"

typedef struct {
    float position_min;
    float position_max;
    float velocity_min;
    float velocity_max;
    float torque_min;
    float torque_max;
    float kp_max;
    float kd_max;
} DMMotorLimits_t;

typedef struct {
    FDCAN_HandleTypeDef *can;
    uint16_t motor_id;
    uint16_t feedback_id;
    DMMotorLimits_t limits;
} DMMotorConfig_t;

typedef struct {
    Motor_t base;
    FDCAN_HandleTypeDef *can;
    uint16_t motor_id;
    uint16_t feedback_id;
    DMMotorLimits_t limits;
    MotorStatus_t status;
    uint8_t feedback_data[8];
    uint8_t has_feedback;
} DMMotor_t;

/* motor_id is the node ID (for example 8), not the 0x108 position-mode CAN ID. */
DMMotorConfig_t DMMotor_DM4310Config(FDCAN_HandleTypeDef *can, uint16_t motor_id,
                                    uint16_t feedback_id);
bool DMMotor_Init(DMMotor_t *motor, const DMMotorConfig_t *config);
/* Supports MIT, torque feedforward, position-speed, and velocity modes. */
bool DMMotor_SendCommand(DMMotor_t *motor, const MotorCmd_t *command);
bool DMMotor_OnFrame(DMMotor_t *motor, uint16_t id,
                     const uint8_t *data, uint8_t length);

#endif

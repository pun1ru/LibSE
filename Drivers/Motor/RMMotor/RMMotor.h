#ifndef RMMOTOR_H
#define RMMOTOR_H

#include "can_comm.h"
#include "motor.h"

typedef struct DJIMotor DJIMotor_t;

typedef struct {
    can_t *can;
    int16_t current[4];
    DJIMotor_t *motors[4];
} DJIMotorGroup_t;

struct DJIMotor {
    Motor_t base;
    DJIMotorGroup_t *group;
    float reduction_ratio;
    int32_t encoder_total;
    uint16_t last_encoder;
    int16_t measured_current;
    uint8_t first_frame;
};

/* One group controls DJI 2006/3508 IDs 1..4 with CAN TX ID 0x200. */
driver_status_t DJIMotorGroup_Init(DJIMotorGroup_t *group, can_t *can);
driver_status_t DJIMotor_Init(DJIMotor_t *motor, DJIMotorGroup_t *group,
                           motor_type_t type, uint8_t motor_id, float reduction_ratio);
driver_status_t DJIMotor_OnFrame(DJIMotor_t *motor, const can_frame_t *frame);

#endif

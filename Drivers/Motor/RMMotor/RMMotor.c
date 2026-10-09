#include "RMMotor.h"

#include <stddef.h>
#include <string.h>

#define RM_ENCODER_COUNTS 8192
#define RM_TWO_PI 6.283185307179586f

static uint8_t RMMotor_Slot(uint8_t motor_id)
{
    return (uint8_t)((motor_id - 1U) % 4U);
}

static int16_t RMMotor_Setpoint(const RMMotor_t *motor)
{
    float value = motor->base.motorcmd.current;
    float limit = motor->motor_config.motor_type == RM_C610 ? RM_C610_CURRENT_LIMIT :
                  motor->motor_config.motor_type == RM_C620 ? RM_C620_CURRENT_LIMIT :
                  motor->motor_config.mode == RM_VOLTAGE ? RM_GM6020_VOLTAGE_LIMIT :
                  RM_GM6020_CURRENT_LIMIT;

    if (value > limit) {
        value = limit;
    } else if (value < -limit) {
        value = -limit;
    }
    return (int16_t)value;
}

void RMMotor_SendGroupCommand(RMMotor_t *motor)
{
    RMMotorGroup_t *group = motor->group;
    uint8_t data[8] = {0};

    for (uint8_t slot = 0U; slot < 4U; ++slot) {
        RMMotor_t *member = group->motors[slot];
        if (member != NULL) {
            int16_t value = RMMotor_Setpoint(member);
            data[slot * 2U] = (uint8_t)((uint16_t)value >> 8);
            data[slot * 2U + 1U] = (uint8_t)value;
        }
    }
    (void)CANTransmit_Enqueue(group->can, group->tx_id, data);
}

static void RMMotor_SendFromBase(Motor_t *base)
{
    RMMotor_SendGroupCommand((RMMotor_t *)base);
}

static void RMMotor_Disable(Motor_t *base)
{
    RMMotor_t *motor = (RMMotor_t *)base;
    motor->base.motorcmd.current = 0.0f;
    motor->base.feedback.status = MOTOR_DISABLED;
    RMMotor_SendGroupCommand(motor);
}

void RMMotorGroup_Init(RMMotorGroup_t *group, FDCAN_HandleTypeDef *can,
                       uint16_t tx_id)
{
    memset(group, 0, sizeof(*group));
    group->can = can;
    group->tx_id = tx_id;
}

void RMMotor_Init(RMMotor_t *motor, RMMotorGroup_t *group,
                  const RMMotorConfig_t *config)
{
    memset(motor, 0, sizeof(*motor));
    motor->motor_config = *config;
    motor->group = group;
    motor->base.feedback.status = MOTOR_NO_FEEDBACK;
    motor->base.motorcmd.control_mode = MODE_CURRENT;
    motor->base.motorops.disable = RMMotor_Disable;
    motor->base.motorops.protect = RMMotor_Disable;
    motor->base.motorops.sendcommand = RMMotor_SendFromBase;
    motor->base.motorops.sendgroupcommand = RMMotor_SendFromBase;
    group->motors[RMMotor_Slot(config->motor_id)] = motor;
}

void RMMotor_Feedback(RMMotor_t *motor, const uint8_t data[8], uint16_t rec_id)
{
    motor->base.feedback.timestamp++;
    uint16_t feedback_id = motor->motor_config.motor_type == RM_GM6020 ?
                           (uint16_t)(0x204U + motor->motor_config.motor_id) :
                           (uint16_t)(0x200U + motor->motor_config.motor_id);
    if (rec_id != feedback_id) {
        return;
    }

    uint16_t encoder = ((uint16_t)data[0] << 8) | data[1];
    if (encoder >= RM_ENCODER_COUNTS) {
        return;
    }

    int16_t speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    Feedback_t *feedback = &motor->base.feedback;
    feedback->position = (float)encoder * RM_TWO_PI / RM_ENCODER_COUNTS;
    feedback->abs_angle = feedback->position;
    feedback->velocity = (float)speed_rpm;
    feedback->omega = feedback->velocity * RM_TWO_PI / 60.0f;
    if (motor->motor_config.motor_type == RM_C620) {
        feedback->temp = (float)data[6];
    } else if (motor->motor_config.motor_type == RM_GM6020) {
        feedback->temp = (float)data[7];
    }
    feedback->error_id = 0U;
    feedback->status = MOTOR_ENABLED;
}

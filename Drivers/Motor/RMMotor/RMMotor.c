#include "DJI_Motor.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DJI_ENCODER_COUNTS 8192.0f
#define DJI_TWO_PI 6.283185307179586f

static driver_status_t DJIMotorGroup_Send(DJIMotorGroup_t *group)
{
    can_frame_t frame = {0};
    uint8_t slot;
    frame.id = 0x200U;
    frame.length = 8U;
    for (slot = 0U; slot < 4U; ++slot) {
        frame.data[slot * 2U] = (uint8_t)(group->current[slot] >> 8);
        frame.data[slot * 2U + 1U] = (uint8_t)group->current[slot];
    }
    return Can_Send(group->can, &frame);
}

static driver_status_t DJIMotor_Enable(Motor_t *base)
{
    (void)base;
    return DRIVER_OK;
}

static driver_status_t DJIMotor_Disable(Motor_t *base)
{
    DJIMotor_t *motor = (DJIMotor_t *)base;
    int16_t previous = motor->group->current[base->id - 1U];
    driver_status_t result;
    motor->group->current[base->id - 1U] = 0;
    result = DJIMotorGroup_Send(motor->group);
    if (result != DRIVER_OK) {
        motor->group->current[base->id - 1U] = previous;
    }
    return result;
}

static driver_status_t DJIMotor_ClearError(Motor_t *base)
{
    (void)base;
    return DRIVER_NOT_SUPPORTED;
}

static driver_status_t DJIMotor_SendCommand(Motor_t *base, const motor_cmd_t *command)
{
    DJIMotor_t *motor = (DJIMotor_t *)base;
    float current;
    int16_t previous;
    driver_status_t result;
    float limit = base->type == MOTOR_TYPE_DJI3508 ? 15999.0f : 9999.0f;
    if (command->control_mode != MOTOR_MODE_CURRENT) {
        return DRIVER_NOT_SUPPORTED;
    }
    if (!isfinite(command->current)) {
        return DRIVER_INVALID_ARGUMENT;
    }
    current = command->current;
    if (current > limit) {
        current = limit;
    } else if (current < -limit) {
        current = -limit;
    }
    previous = motor->group->current[base->id - 1U];
    motor->group->current[base->id - 1U] = (int16_t)current;
    result = DJIMotorGroup_Send(motor->group);
    if (result != DRIVER_OK) {
        motor->group->current[base->id - 1U] = previous;
    }
    return result;
}

static const MotorOps_t dji_motor_ops = {
    DJIMotor_Enable, DJIMotor_Disable, DJIMotor_ClearError, DJIMotor_SendCommand
};

driver_status_t DJIMotorGroup_Init(DJIMotorGroup_t *group, can_t *can)
{
    if (group == NULL || can == NULL) {
        return DRIVER_INVALID_ARGUMENT;
    }
    memset(group, 0, sizeof(*group));
    group->can = can;
    return DRIVER_OK;
}

driver_status_t DJIMotor_Init(DJIMotor_t *motor, DJIMotorGroup_t *group,
                           motor_type_t type, uint8_t motor_id, float reduction_ratio)
{
    driver_status_t result;
    if (motor == NULL || group == NULL || group->can == NULL ||
        (type != MOTOR_TYPE_DJI2006 && type != MOTOR_TYPE_DJI3508) ||
        motor_id < 1U || motor_id > 4U ||
        !isfinite(reduction_ratio) || reduction_ratio <= 0.0f ||
        group->motors[motor_id - 1U] != NULL) {
        return DRIVER_INVALID_ARGUMENT;
    }
    result = Motor_Init(&motor->base, type, motor_id, &dji_motor_ops);
    if (result != DRIVER_OK) {
        return result;
    }
    motor->group = group;
    motor->reduction_ratio = reduction_ratio;
    motor->encoder_total = 0;
    motor->last_encoder = 0U;
    motor->measured_current = 0;
    motor->first_frame = 1U;
    group->motors[motor_id - 1U] = motor;
    return DRIVER_OK;
}

driver_status_t DJIMotor_OnFrame(DJIMotor_t *motor, const can_frame_t *frame)
{
    motor_feedback_t *feedback;
    uint16_t encoder;
    int32_t delta;
    int16_t speed_rpm;
    if (motor == NULL || frame == NULL) {
        return DRIVER_INVALID_ARGUMENT;
    }
    if (frame->extended != 0U || frame->id != 0x200U + motor->base.id) {
        return DRIVER_NOT_SUPPORTED;
    }
    if (frame->length != 8U) {
        return DRIVER_ERROR;
    }
    encoder = ((uint16_t)frame->data[0] << 8) | frame->data[1];
    if (encoder >= 8192U) {
        return DRIVER_ERROR;
    }
    if (motor->first_frame != 0U) {
        motor->encoder_total = encoder;
        motor->first_frame = 0U;
    } else {
        delta = (int32_t)encoder - motor->last_encoder;
        if (delta > 4096) {
            delta -= 8192;
        } else if (delta < -4096) {
            delta += 8192;
        }
        motor->encoder_total += delta;
    }
    motor->last_encoder = encoder;
    speed_rpm = (int16_t)(((uint16_t)frame->data[2] << 8) | frame->data[3]);
    motor->measured_current = (int16_t)(((uint16_t)frame->data[4] << 8) | frame->data[5]);
    feedback = &motor->base.feedback;
    feedback->pos = (float)encoder * DJI_TWO_PI / DJI_ENCODER_COUNTS / motor->reduction_ratio;
    feedback->multi_pos = (float)motor->encoder_total * DJI_TWO_PI /
                          DJI_ENCODER_COUNTS / motor->reduction_ratio;
    feedback->vel = (float)speed_rpm * DJI_TWO_PI / 60.0f / motor->reduction_ratio;
    feedback->temp = (float)frame->data[6];
    return DRIVER_OK;
}

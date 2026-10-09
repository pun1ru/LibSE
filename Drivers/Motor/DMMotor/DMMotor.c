#include "DMMotor.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "DMJ4310.h"

#define DM_TWO_PI 6.283185307179586f

//uint16_t与float之间的映射函数
static uint16_t DMMotor_Encode(float value, float min, float max, uint8_t bits)
{
    return (uint16_t)((value - min) * (float)((1U << bits) - 1U) / (max - min));
}

static float DMMotor_Decode(uint16_t value, float min, float max, uint8_t bits)
{
    return (float)value * (max - min) / (float)((1U << bits) - 1U) + min;
}

//发送帧，改成加到queue里面
static bool DMMotor_SendFrame(DMMotor_t *motor, uint16_t id, uint8_t data[8])
{
    return fdcanx_send_data(motor->can, id, data, 8U) == 0U;
}

//再封装一层的意义是什么，code直接写啊，然后函数命名指向发送的功能
static bool DMMotor_SendSpecial(DMMotor_t *motor, uint8_t code)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                       0xFFU, 0xFFU, 0xFFU, code};
    return DMMotor_SendFrame(motor, motor->motor_id, data);
}

//motor状态要根据feedback来判断啊
static void DMMotor_Enable(Motor_t *base)
{
    DMMotor_t *motor = (DMMotor_t *)base;
    if (DMMotor_SendSpecial(motor, 0xFCU)) {
        motor->status = MOTOR_ENABLED;
    }
}

static void DMMotor_Disable(Motor_t *base)
{
    DMMotor_t *motor = (DMMotor_t *)base;
    if (DMMotor_SendSpecial(motor, 0xFDU)) {
        motor->status = MOTOR_DISABLED;
    } else {
        motor->status = MOTOR_FAULT;
    }
}

static void DMMotor_Protect(Motor_t *base)
{
    DMMotor_t *motor = (DMMotor_t *)base;
    motor->status = MOTOR_FAULT;
    (void)DMMotor_SendSpecial(motor, 0xFDU);
}

static void DMMotor_ClearError(Motor_t *base)
{
    DMMotor_t *motor = (DMMotor_t *)base;
    if (DMMotor_SendSpecial(motor, 0xFBU)) {
        motor->status = MOTOR_DISABLED;
        base->feedback.error_id = 0U;
    }
}

static void DMMotor_GetFeedback(Motor_t *base)
{
    DMMotor_t *motor = (DMMotor_t *)base;
    Feedback_t *feedback = &base->feedback;
    const uint8_t *data = motor->feedback_data;
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;
    float previous_position;
    float delta;
    uint8_t state;

    if (motor->has_feedback == 0U) {
        return;
    }

    position_raw = ((uint16_t)data[1] << 8) | data[2];
    velocity_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];
    previous_position = feedback->position;
    feedback->position = DMMotor_Decode(position_raw, motor->limits.position_min,
                                        motor->limits.position_max, 16U);
    feedback->abs_angle = feedback->position;
    if (feedback->abs_angle < 0.0f) {
        feedback->abs_angle += DM_TWO_PI;
    }
    if (motor->has_feedback == 1U) {
        feedback->multi_angle = feedback->position;
    } else {
        delta = feedback->position - previous_position;
        if (delta > DM_TWO_PI / 2.0f) {
            delta -= DM_TWO_PI;
        } else if (delta < -DM_TWO_PI / 2.0f) {
            delta += DM_TWO_PI;
        }
        feedback->multi_angle += delta;
    }
    feedback->velocity = DMMotor_Decode(velocity_raw, motor->limits.velocity_min,
                                        motor->limits.velocity_max, 12U);
    feedback->omega = feedback->velocity;
    feedback->torque = DMMotor_Decode(torque_raw, motor->limits.torque_min,
                                      motor->limits.torque_max, 12U);
    feedback->temp = (float)data[7];
    state = data[0] >> 4;
    feedback->state = state;
    feedback->error_id = state >= 8U ? state : 0U;
    if (feedback->error_id != 0U) {
        motor->status = MOTOR_FAULT;
    } else if (state == 1U) {
        motor->status = MOTOR_ENABLED;
    } else if (state == 0U) {
        motor->status = MOTOR_DISABLED;
    }
    motor->has_feedback = 2U;
}

DMMotorConfig_t DMMotor_DM4310Config(FDCAN_HandleTypeDef *can,
                                    uint16_t motor_id, uint16_t feedback_id)
{
    DMMotorConfig_t config = {0};
    config.can = can;
    config.motor_id = motor_id;
    config.feedback_id = feedback_id;
    config.limits.position_min = P_MIN;
    config.limits.position_max = P_MAX;
    config.limits.velocity_min = V_MIN;
    config.limits.velocity_max = V_MAX;
    config.limits.torque_min = T_MIN;
    config.limits.torque_max = T_MAX;
    config.limits.kp_max = KP_MAX;
    config.limits.kd_max = KD_MAX;
    return config;
}

bool DMMotor_Init(DMMotor_t *motor, const DMMotorConfig_t *config)
{
    if (motor == NULL || config == NULL || config->can == NULL ||
        config->motor_id == 0U || config->motor_id > 15U ||
        config->feedback_id > 0x7FFU ||
        !isfinite(config->limits.position_min) ||
        !isfinite(config->limits.position_max) ||
        config->limits.position_min >= config->limits.position_max ||
        !isfinite(config->limits.velocity_min) ||
        !isfinite(config->limits.velocity_max) ||
        config->limits.velocity_min >= config->limits.velocity_max ||
        !isfinite(config->limits.torque_min) ||
        !isfinite(config->limits.torque_max) ||
        config->limits.torque_min >= config->limits.torque_max ||
        !isfinite(config->limits.kp_max) || config->limits.kp_max <= 0.0f ||
        !isfinite(config->limits.kd_max) || config->limits.kd_max <= 0.0f) {
        return false;
    }

    memset(motor, 0, sizeof(*motor));
    motor->can = config->can;
    motor->motor_id = config->motor_id;
    motor->feedback_id = config->feedback_id;
    motor->limits = config->limits;
    motor->status = MOTOR_DISABLED;
    motor->base.motorops.enable = DMMotor_Enable;
    motor->base.motorops.disable = DMMotor_Disable;
    motor->base.motorops.protect = DMMotor_Protect;
    motor->base.motorops.clearerror = DMMotor_ClearError;
    motor->base.motorops.getfeedback = DMMotor_GetFeedback;
    return true;
}

bool DMMotor_SendCommand(DMMotor_t *motor, const MotorCmd_t *command)
{
    uint8_t data[8] = {0};
    uint16_t id;
    const DMMotorLimits_t *limits;

    if (motor == NULL || command == NULL || motor->can == NULL ||
        motor->status != MOTOR_ENABLED) {
        return false;
    }
    limits = &motor->limits;
    switch (command->control_mode) {
    case MODE_TORQUE:
    case MODE_MIT: {
        bool torque_only = command->control_mode == MODE_TORQUE;
        float target_position = torque_only ? 0.0f : command->position;
        float target_velocity = torque_only ? 0.0f : command->velocity;
        float target_kp = torque_only ? 0.0f : command->kp;
        float target_kd = torque_only ? 0.0f : command->kd;
        uint16_t position;
        uint16_t velocity;
        uint16_t kp;
        uint16_t kd;
        uint16_t torque;
        if (!isfinite(target_position) ||
            target_position < limits->position_min ||
            target_position > limits->position_max ||
            !isfinite(target_velocity) ||
            target_velocity < limits->velocity_min ||
            target_velocity > limits->velocity_max ||
            !isfinite(command->torque) ||
            command->torque < limits->torque_min ||
            command->torque > limits->torque_max ||
            !isfinite(target_kp) || target_kp < 0.0f ||
            target_kp > limits->kp_max ||
            !isfinite(target_kd) || target_kd < 0.0f ||
            target_kd > limits->kd_max) {
            return false;
        }
        position = DMMotor_Encode(target_position, limits->position_min,
                                  limits->position_max, 16U);
        velocity = DMMotor_Encode(target_velocity, limits->velocity_min,
                                  limits->velocity_max, 12U);
        kp = DMMotor_Encode(target_kp, 0.0f, limits->kp_max, 12U);
        kd = DMMotor_Encode(target_kd, 0.0f, limits->kd_max, 12U);
        torque = DMMotor_Encode(command->torque, limits->torque_min,
                                limits->torque_max, 12U);
        data[0] = (uint8_t)(position >> 8);
        data[1] = (uint8_t)position;
        data[2] = (uint8_t)(velocity >> 4);
        data[3] = (uint8_t)((velocity << 4) | (kp >> 8));
        data[4] = (uint8_t)kp;
        data[5] = (uint8_t)(kd >> 4);
        data[6] = (uint8_t)((kd << 4) | (torque >> 8));
        data[7] = (uint8_t)torque;
        id = motor->motor_id;
        break;
    }
    case MODE_POSITION:
        if (!isfinite(command->position) ||
            !isfinite(command->velocity) ||
            command->velocity < 0.0f ||
            command->velocity > limits->velocity_max) {
            return false;
        }
        memcpy(data, &command->position, sizeof(float));
        memcpy(data + 4, &command->velocity, sizeof(float));
        id = motor->motor_id + 0x100U;
        break;
    case MODE_VELOCITY:
        if (!isfinite(command->velocity) ||
            command->velocity < limits->velocity_min ||
            command->velocity > limits->velocity_max) {
            return false;
        }
        memcpy(data, &command->velocity, sizeof(float));
        id = motor->motor_id + 0x200U;
        break;
    default:
        return false;
    }

    if (!DMMotor_SendFrame(motor, id, data)) {
        return false;
    }
    motor->base.motorcmd = *command;
    return true;
}

bool DMMotor_OnFrame(DMMotor_t *motor, uint16_t id,
                     const uint8_t *data, uint8_t length)
{
    if (motor == NULL || data == NULL || id != motor->feedback_id ||
        length != 8U || (data[0] & 0x0FU) != motor->motor_id ||
        motor->base.motorops.getfeedback == NULL) {
        return false;
    }
    memcpy(motor->feedback_data, data, 8U);
    motor->has_feedback = motor->has_feedback == 0U ? 1U : 2U;
    motor->base.motorops.getfeedback(&motor->base);
    return true;
}

#include "DMMotor.h"

#include <stddef.h>
#include <string.h>

//uint16_t与float之间的映射函数
static uint16_t DMMotor_Encode(float value, float min, float max, uint8_t bits)
{
    if (value < min)
    {
        value = min;
    }
    else if (value > max)
    {
        value = max;
    }
    return (uint16_t)((value - min) * (float)((1U << bits) - 1U) / (max - min));
}

static float DMMotor_Decode(uint16_t value, float min, float max, uint8_t bits)
{
    return (float)value * (max - min) / (float)((1U << bits) - 1U) + min;
}

static void DMMotor_SendFrame(DMMotor_t *motor, uint16_t id, const uint8_t data[8])
{
    (void)CANTransmit_Enqueue(motor->base.can, id, data);
}

static void DMMotor_SendEnable(Motor_t *base)
{
    DMMotor_t *motor = (DMMotor_t *)base;
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                       0xFFU, 0xFFU, 0xFFU, 0xFCU};
    DMMotor_SendFrame(motor, motor->motor_config.motor_id, data);
}

static void DMMotor_SendDisable(Motor_t *base)
{
    DMMotor_t *motor = (DMMotor_t *)base;
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                       0xFFU, 0xFFU, 0xFFU, 0xFDU};
    DMMotor_SendFrame(motor, motor->motor_config.motor_id, data);
}

static void DMMotor_SendClearError(Motor_t *base)
{
    DMMotor_t *motor = (DMMotor_t *)base;
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                       0xFFU, 0xFFU, 0xFFU, 0xFBU};
    DMMotor_SendFrame(motor, motor->motor_config.motor_id, data);
}

static void DMMotor_SendFromBase(Motor_t *base)
{
    DMMotor_SendCommand((DMMotor_t *)base);
}

static void DMMotor_FeedbackFromBase(Motor_t *base, const uint8_t *data,
                                     uint16_t rec_id)
{
    DMMotor_Feedback((DMMotor_t *)base, data, rec_id);
}

void DMMotor_Feedback(DMMotor_t *motor, const uint8_t data[8],uint16_t rec_id)
{
    if(rec_id != motor->motor_config.feedback_id ||
       (data[0] & 0x0FU) != (motor->motor_config.motor_id & 0x0FU))
    {
        return;
    }
    Feedback_t *feedback = &motor->base.feedback;
    feedback->timestamp++;
    const LSB *limits = &motor->motor_config.lsb;
    uint16_t position_raw = ((uint16_t)data[1] << 8) | data[2];
    uint16_t velocity_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    uint16_t torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];
    feedback->encoder = position_raw;
    /* POS 按 [P_MIN, P_MAX] 线性映射为 rad，zero_point 是同一映射下的原始值 */
    float position_rad = DMMotor_Decode(position_raw, limits->p_min,
                                        limits->p_max, 16U) -
                         DMMotor_Decode(motor->base.zero_point, limits->p_min,
                                        limits->p_max, 16U);
    feedback->multi_angle_rad = position_rad;
    Motor_SetAbsAngle(feedback, position_rad);
    feedback->omega_rads = DMMotor_Decode(velocity_raw, limits->v_min,
                                          limits->v_max, 12U);
    feedback->velocity_rpm = feedback->omega_rads * MOTOR_RADS_TO_RPM;
    feedback->torque_nm = DMMotor_Decode(torque_raw, limits->t_min,
                                         limits->t_max, 12U);
    feedback->temp = (float)data[7];   /* T_Rotor 线圈温度 */
    uint8_t status_code = data[0] >> 4;
    feedback->error_id = status_code > 1U ? status_code : 0U;
    switch (feedback->error_id) {
    case 0x0:
        feedback->status = status_code == 1U ? MOTOR_ENABLED : MOTOR_DISABLED;
        break;
    case 0x8:
        feedback->status = MOTOR_OVERVOLTAGE;
        break;
    case 0x9:
        feedback->status = MOTOR_UNDERVOLTAGE;
        break;
    case 0xA:
        feedback->status = MOTOR_OVERCURRENT;
        break;
    case 0xB:
        feedback->status = MOTOR_MOS_OVERTEMP;
        break;
    case 0xC:
        feedback->status = MOTOR_COIL_OVERTEMP;
        break;
    case 0xD:
        feedback->status = MOTOR_COMM_LOST;
        break;
    case 0xE:
        feedback->status = MOTOR_OVERLOAD;
        break;
    default:
        feedback->status = MOTOR_FAULT;
        break;
    }
}

void DMMotor_Init(DMMotor_t *motor, FDCAN_HandleTypeDef *can,
                  const MotorConfig_t *config)
{
    MotorConfig_t saved_config = *config;
    switch (saved_config.motor_type) {
    case DM_4310:
        saved_config.lsb.p_min = DM4310_P_MIN;
        saved_config.lsb.p_max = DM4310_P_MAX;
        saved_config.lsb.v_min = DM4310_V_MIN;
        saved_config.lsb.v_max = DM4310_V_MAX;
        saved_config.lsb.kp_min = DM4310_KP_MIN;
        saved_config.lsb.kp_max = DM4310_KP_MAX;
        saved_config.lsb.kd_min = DM4310_KD_MIN;
        saved_config.lsb.kd_max = DM4310_KD_MAX;
        saved_config.lsb.t_min = DM4310_T_MIN;
        saved_config.lsb.t_max = DM4310_T_MAX;
        break;
    case DM_6006:
        saved_config.lsb.p_min = DM6006_P_MIN;
        saved_config.lsb.p_max = DM6006_P_MAX;
        saved_config.lsb.v_min = DM6006_V_MIN;
        saved_config.lsb.v_max = DM6006_V_MAX;
        saved_config.lsb.kp_min = DM6006_KP_MIN;
        saved_config.lsb.kp_max = DM6006_KP_MAX;
        saved_config.lsb.kd_min = DM6006_KD_MIN;
        saved_config.lsb.kd_max = DM6006_KD_MAX;
        saved_config.lsb.t_min = DM6006_T_MIN;
        saved_config.lsb.t_max = DM6006_T_MAX;
        break;
    default:
        return;
    }

    memset(motor, 0, sizeof(*motor));
    motor->motor_config = saved_config;
    Motor_BaseInit(&motor->base, can, config->zero_point, config->inverse);
    motor->base.motorops.enable = DMMotor_SendEnable;
    motor->base.motorops.disable = DMMotor_SendDisable;
    motor->base.motorops.protect = DMMotor_SendDisable;
    motor->base.motorops.sendcommand = DMMotor_SendFromBase;
    motor->base.motorops.clearerror = DMMotor_SendClearError;
    motor->base.motorops.getfeedback = DMMotor_FeedbackFromBase;
}

void DMMotor_SendCommand(DMMotor_t *motor)
{
    uint8_t data[8] = {0};
    uint16_t id;
    const LSB *limits;
    const MotorCmd_t *command;

    limits = &motor->motor_config.lsb;
    command = &motor->base.motorcmd;
    switch (command->control_mode) {
    case MODE_TORQUE:
    case MODE_MIT: {
        bool torque_only = command->control_mode == MODE_TORQUE;
        float target_position = torque_only ? 0.0f : command->position;
        float target_velocity = torque_only ? 0.0f : command->velocity;
        float target_kp = torque_only ? 0.0f : command->kp;
        float target_kd = torque_only ? 0.0f : command->kd;
        float target_torque = motor->base.inverse ? -command->torque :
                              command->torque;
        uint16_t position = DMMotor_Encode(target_position, limits->p_min,
                                  limits->p_max, 16U);
        uint16_t velocity = DMMotor_Encode(target_velocity, limits->v_min,
                                  limits->v_max, 12U);
        uint16_t kp = DMMotor_Encode(target_kp, limits->kp_min, limits->kp_max, 12U);
        uint16_t kd = DMMotor_Encode(target_kd, limits->kd_min, limits->kd_max, 12U);
        uint16_t torque = DMMotor_Encode(target_torque, limits->t_min,
                                limits->t_max, 12U);
        data[0] = (uint8_t)(position >> 8);
        data[1] = (uint8_t)position;
        data[2] = (uint8_t)(velocity >> 4);
        data[3] = (uint8_t)((velocity << 4) | (kp >> 8));
        data[4] = (uint8_t)kp;
        data[5] = (uint8_t)(kd >> 4);
        data[6] = (uint8_t)((kd << 4) | (torque >> 8));
        data[7] = (uint8_t)torque;
        id = motor->motor_config.motor_id;
        break;
    }
    case MODE_POSITION:
        memcpy(data, &command->position, sizeof(float));
        memcpy(data + 4, &command->velocity, sizeof(float));
        id = motor->motor_config.motor_id + 0x100U;
        break;
    case MODE_VELOCITY:
        memcpy(data, &command->velocity, sizeof(float));
        id = motor->motor_config.motor_id + 0x200U;
        break;
    default:
        return;
    }
    DMMotor_SendFrame(motor, id, data);
}

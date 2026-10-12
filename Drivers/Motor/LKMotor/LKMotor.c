#include "LKMotor.h"

#include <string.h>

#define LK_CMD_MOTOR_OFF           0x80U
#define LK_CMD_MOTOR_STOP          0x81U
#define LK_CMD_MOTOR_RUN           0x88U
#define LK_CMD_CLEAR_ERROR         0x9BU
#define LK_CMD_READ_ERROR          0x9AU
#define LK_CMD_READ_STATE          0x9CU
#define LK_CMD_READ_MULTI_ANGLE    0x92U
#define LK_CMD_READ_SINGLE_ANGLE   0x94U
#define LK_CMD_IQ_CONTROL          0xA1U
#define LK_CMD_SPEED_CONTROL       0xA2U
#define LK_CMD_POSITION_CONTROL    0xA3U
#define LK_CMD_POSITION_LIMITED    0xA4U
#define LK_PI                      3.141592653589793f
#define LK_DEGREES_PER_REVOLUTION  360.0f
#define LK_RAD_PER_DEGREE          (LK_PI / 180.0f)
#define LK_STATE_REQUEST           0U
#define LK_ANGLE_REQUEST           1U
#define LK_ERROR_REQUEST           2U
#define LK_ERROR_UNDERVOLTAGE      (1U << 0U)
#define LK_ERROR_OVERVOLTAGE       (1U << 1U)
#define LK_ERROR_MOS_OVERTEMP      (1U << 2U)
#define LK_ERROR_COIL_OVERTEMP     (1U << 3U)
#define LK_ERROR_OVERCURRENT       (1U << 4U)
#define LK_ERROR_COMM_LOST         (1U << 7U)
#define LK_MOTOR_OFF_STATE         0x10U
#define LK_TWO_PI                  (LK_DEGREES_PER_REVOLUTION * LK_RAD_PER_DEGREE)

static uint16_t LKMotor_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void LKMotor_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static float LKMotor_Limit(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static void LKMotor_SendFrame(LKMotor_t *motor, const uint8_t data[8])
{
    (void)CANTransmit_Enqueue(motor->motor_config.can,
                              (uint16_t)(LK_CAN_COMMAND_ID_BASE + motor->motor_config.motor_id),
                              data);
}

static void LKMotor_SendSimple(Motor_t *base, uint8_t command)
{
    LKMotor_t *motor = (LKMotor_t *)base;
    uint8_t data[8] = {0};
    data[0] = command;
    LKMotor_SendFrame(motor, data);
}

static void LKMotor_Enable(Motor_t *base)
{
    LKMotor_SendSimple(base, LK_CMD_MOTOR_RUN);
}

static void LKMotor_Disable(Motor_t *base)
{
    LKMotor_SendSimple(base, LK_CMD_MOTOR_OFF);
}

static void LKMotor_Protect(Motor_t *base)
{
    LKMotor_SendSimple(base, LK_CMD_MOTOR_STOP);
}

static void LKMotor_ClearError(Motor_t *base)
{
    LKMotor_SendSimple(base, LK_CMD_CLEAR_ERROR);
}

static void LKMotor_SendFromBase(Motor_t *base)
{
    LKMotor_SendCommand((LKMotor_t *)base);
}

static void LKMotor_RequestFromBase(Motor_t *base)
{
    LKMotor_RequestFeedback((LKMotor_t *)base);
}

static void LKMotor_FeedbackFromBase(Motor_t *base, const uint8_t *data,
                                     uint16_t rec_id)
{
    LKMotor_Feedback((LKMotor_t *)base, data, rec_id);
}

void LKMotor_Init(LKMotor_t *motor, const LKMotorConfig_t *config)
{
    memset(motor, 0, sizeof(*motor));
    motor->motor_config = *config;
    motor->base.zero_point = config->zero_point;
    motor->base.inverse = config->inverse;
    motor->base.feedback.status = MOTOR_NO_FEEDBACK;
    motor->base.motorcmd.control_mode = MODE_CURRENT;
    motor->base.motorops.enable = LKMotor_Enable;
    motor->base.motorops.disable = LKMotor_Disable;
    motor->base.motorops.protect = LKMotor_Protect;
    motor->base.motorops.sendcommand = LKMotor_SendFromBase;
    motor->base.motorops.requestfeedback = LKMotor_RequestFromBase;
    motor->base.motorops.clearerror = LKMotor_ClearError;
    motor->base.motorops.getfeedback = LKMotor_FeedbackFromBase;
}

void LKMotor_SendCommand(LKMotor_t *motor)
{
    const MotorCmd_t *command = &motor->base.motorcmd;
    uint8_t data[8] = {0};
    int32_t setpoint;

    switch (command->control_mode)
    {
    case MODE_CURRENT:
        setpoint = (int32_t)LKMotor_Limit(motor->base.inverse ?
                                        -command->current : command->current,
                                        LK_IQ_COMMAND_LIMIT);
        data[0] = LK_CMD_IQ_CONTROL;
        data[4] = (uint8_t)setpoint;
        data[5] = (uint8_t)((uint32_t)setpoint >> 8);
        break;
    case MODE_VELOCITY:
        setpoint = (int32_t)(command->velocity * LK_SPEED_COMMAND_SCALE /
                             LK_RAD_PER_DEGREE);
        data[0] = LK_CMD_SPEED_CONTROL;
        LKMotor_WriteU32(&data[4], (uint32_t)setpoint);
        break;
    case MODE_POSITION:
        setpoint = (int32_t)(command->position /
                             (LK_ANGLE_DEGREE_PER_LSB * LK_RAD_PER_DEGREE));
        if (motor->motor_config.max_speed_dps != 0U)
        {
            data[0] = LK_CMD_POSITION_LIMITED;
            data[2] = (uint8_t)motor->motor_config.max_speed_dps;
            data[3] = (uint8_t)(motor->motor_config.max_speed_dps >> 8);
        }
        else
        {
            data[0] = LK_CMD_POSITION_CONTROL;
        }
        LKMotor_WriteU32(&data[4], (uint32_t)setpoint);
        break;
    default:
        return;
    }
    LKMotor_SendFrame(motor, data);
}

void LKMotor_RequestFeedback(LKMotor_t *motor)
{
    uint8_t data[8] = {0};
    if (motor->next_request == LK_STATE_REQUEST)
    {
        data[0] = LK_CMD_READ_STATE;
        motor->next_request = LK_ANGLE_REQUEST;
    }
    else if (motor->next_request == LK_ANGLE_REQUEST)
    {
        data[0] = LK_CMD_READ_MULTI_ANGLE;
        motor->next_request = LK_ERROR_REQUEST;
    }
    else
    {
        data[0] = LK_CMD_READ_ERROR;
        motor->next_request = LK_STATE_REQUEST;
    }
    LKMotor_SendFrame(motor, data);
}

void LKMotor_Feedback(LKMotor_t *motor, const uint8_t data[8], uint16_t rec_id)
{
    if (rec_id != (uint16_t)(LK_CAN_FEEDBACK_ID_BASE + motor->motor_config.motor_id))
    {
        return;
    }

    Feedback_t *feedback = &motor->base.feedback;
    switch (data[0])
    {
    case LK_CMD_READ_ERROR:
    case LK_CMD_CLEAR_ERROR:
        feedback->temp = (float)(int8_t)data[1];
        feedback->error_id = data[7];
        if ((data[7] & LK_ERROR_UNDERVOLTAGE) != 0U)
        {
            feedback->status = MOTOR_UNDERVOLTAGE;
        }
        else if ((data[7] & LK_ERROR_OVERVOLTAGE) != 0U)
        {
            feedback->status = MOTOR_OVERVOLTAGE;
        }
        else if ((data[7] & LK_ERROR_MOS_OVERTEMP) != 0U)
        {
            feedback->status = MOTOR_MOS_OVERTEMP;
        }
        else if ((data[7] & LK_ERROR_COIL_OVERTEMP) != 0U)
        {
            feedback->status = MOTOR_COIL_OVERTEMP;
        }
        else if ((data[7] & LK_ERROR_OVERCURRENT) != 0U)
        {
            feedback->status = MOTOR_OVERCURRENT;
        }
        else if ((data[7] & LK_ERROR_COMM_LOST) != 0U)
        {
            feedback->status = MOTOR_COMM_LOST;
        }
        else if (data[7] != 0U)
        {
            feedback->status = MOTOR_FAULT;
        }
        else
        {
            feedback->status = data[6] == LK_MOTOR_OFF_STATE ?
                               MOTOR_DISABLED : MOTOR_ENABLED;
        }
        break;
    case LK_CMD_READ_STATE:
    case LK_CMD_IQ_CONTROL:
    case LK_CMD_SPEED_CONTROL:
    case LK_CMD_POSITION_CONTROL:
    case LK_CMD_POSITION_LIMITED:
    {
        int16_t iq = (int16_t)LKMotor_ReadU16(&data[2]);
        int16_t speed = (int16_t)LKMotor_ReadU16(&data[4]);
        uint16_t encoder = LKMotor_ReadU16(&data[6]);
        feedback->current = (float)iq * motor->motor_config.current_per_lsb;
        feedback->velocity = (float)speed * LK_SPEED_DPS_PER_LSB *
                             LK_RAD_PER_DEGREE;
        feedback->omega = feedback->velocity;
        feedback->encoder = encoder;
        feedback->abs_angle = (float)(uint16_t)(encoder - motor->base.zero_point) *
                              LK_TWO_PI / (float)LK_ENCODER_COUNTS;
        feedback->temp = (float)(int8_t)data[1];
        if (feedback->status == MOTOR_NO_FEEDBACK)
        {
            feedback->status = MOTOR_ENABLED;
        }
        break;
    }
    case LK_CMD_READ_MULTI_ANGLE:
    {
        uint64_t raw = 0U;
        for (uint8_t index = 0U; index < 7U; ++index)
        {
            raw |= (uint64_t)data[index + 1U] << (index * 8U);
        }
        if ((raw & (1ULL << 55U)) != 0U)
        {
            raw |= 0xFF00000000000000ULL;
        }
        feedback->multi_angle = (float)(int64_t)raw *
                                LK_ANGLE_DEGREE_PER_LSB * LK_RAD_PER_DEGREE;
        break;
    }
    case LK_CMD_READ_SINGLE_ANGLE:
    {
        uint32_t angle = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                         ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
        float relative_angle = (float)angle * LK_ANGLE_DEGREE_PER_LSB *
                               LK_RAD_PER_DEGREE -
                               (float)motor->base.zero_point * LK_TWO_PI /
                               (float)LK_ENCODER_COUNTS;
        if (relative_angle < 0.0f)
        {
            relative_angle += LK_TWO_PI;
        }
        else if (relative_angle >= LK_TWO_PI)
        {
            relative_angle -= LK_TWO_PI;
        }
        feedback->abs_angle = relative_angle;
        break;
    }
    default:
        return;
    }
    feedback->timestamp++;
}

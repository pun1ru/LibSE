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
#define LK_DPS_TO_RPM              (60.0f / 360.0f)
#define LK_ERROR_UNDERVOLTAGE      (1U << 0U)
#define LK_ERROR_OVERVOLTAGE       (1U << 1U)
#define LK_ERROR_MOS_OVERTEMP      (1U << 2U)
#define LK_ERROR_COIL_OVERTEMP     (1U << 3U)
#define LK_ERROR_OVERCURRENT       (1U << 4U)
#define LK_ERROR_COMM_LOST         (1U << 7U)
#define LK_MOTOR_OFF_STATE         0x10U

typedef struct {
    float current_per_lsb;      /* iq 的 A/LSB，MG 66/4096，MF 33/4096 */
    float torque_constant;      /* 电流到力矩的系数 N*m/A，直接使用，不再乘减速比 */
    float gear_ratio;
    float iq_per_nm;            /* 1 N*m 对应的 iq 原始值，力矩模式用它换算 */
} LKModelParam_t;

#define LK_IQ_PER_NM(torque_constant, current_per_lsb) (1.0f / ((torque_constant) * (current_per_lsb)))
#define LK_MG6012E_I8_TORQUE_CONSTANT 1.09f

/* 取选型册印的 1.09，换算口径同 serialleg2027（一个固定的 iq / N*m，不乘减速比）
 * 按册内其他数据推算转子端 Kt 约 0.22，乘减速比 8 约 1.75，与 1.09 不一致，实际输出力矩以实测为准 */
static const LKModelParam_t lkModelParams[LK_MODEL_COUNT] =
{
    [LK_MG6012E_I8] = {LK_MG_CURRENT_PER_LSB, LK_MG6012E_I8_TORQUE_CONSTANT, 8.0f,
                       LK_IQ_PER_NM(LK_MG6012E_I8_TORQUE_CONSTANT, LK_MG_CURRENT_PER_LSB)},
};

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
    (void)CANTransmit_Enqueue(motor->base.can,
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

void LKMotor_Init(LKMotor_t *motor, FDCAN_HandleTypeDef *can,
                  const LKMotorConfig_t *config)
{
    const LKModelParam_t *param = &lkModelParams[config->model];

    memset(motor, 0, sizeof(*motor));
    motor->motor_config = *config;
    Motor_BaseInit(&motor->base, can, config->zero_point, config->inverse, param->gear_ratio);
    motor->base.torque_constant = param->torque_constant;
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
    case MODE_TORQUE:
    {
        /* 力矩模式：N*m 按 iq_per_nm 换成 iq，之后与电流模式同一路径 */
        float iq = command->control_mode == MODE_TORQUE ?
                   command->torque * lkModelParams[motor->motor_config.model].iq_per_nm :
                   command->current;
        setpoint = (int32_t)LKMotor_Limit(motor->base.inverse ? -iq : iq,
                                          LK_IQ_COMMAND_LIMIT);
        data[0] = LK_CMD_IQ_CONTROL;
        data[4] = (uint8_t)setpoint;
        data[5] = (uint8_t)((uint32_t)setpoint >> 8);
        break;
    }
    case MODE_VELOCITY:
        /* velocity: rad/s -> 0.01 dps/LSB */
        setpoint = (int32_t)(command->velocity * MOTOR_RAD_TO_DEG *
                             LK_SPEED_COMMAND_SCALE);
        data[0] = LK_CMD_SPEED_CONTROL;
        LKMotor_WriteU32(&data[4], (uint32_t)setpoint);
        break;
    case MODE_POSITION:
        /* position: rad -> 0.01 deg/LSB，最大速度由上位机 Max Speed 限制 */
        setpoint = (int32_t)(command->position * MOTOR_RAD_TO_DEG /
                             LK_ANGLE_DEGREE_PER_LSB);
        data[0] = LK_CMD_POSITION_CONTROL;
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
    data[0] = LK_CMD_READ_MULTI_ANGLE;
    LKMotor_SendFrame(motor, data);
}

void LKMotor_Feedback(LKMotor_t *motor, const uint8_t data[8], uint16_t rec_id)
{
    if (rec_id != motor->motor_config.feedback_id)
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
        float speed_dps = (float)speed * LK_SPEED_DPS_PER_LSB;
        feedback->current = (float)iq * lkModelParams[motor->motor_config.model].current_per_lsb;
        feedback->torque_nm = feedback->current * motor->base.torque_constant;
        feedback->velocity_rpm = speed_dps * LK_DPS_TO_RPM;
        feedback->omega_rads = speed_dps * MOTOR_DEG_TO_RAD;
        feedback->encoder = encoder;
        /* 16 位差值按有符号解释即落在 [-32768, 32767] */
        Motor_SetAbsAngle(feedback,
                          (float)(int16_t)(uint16_t)(encoder - motor->base.zero_point) *
                          MOTOR_TWO_PI / (float)LK_ENCODER_COUNTS);
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
        feedback->multi_angle_rad = (float)(int64_t)raw *
                                    LK_ANGLE_DEGREE_PER_LSB * MOTOR_DEG_TO_RAD;
        break;
    }
    case LK_CMD_READ_SINGLE_ANGLE:
    {
        uint32_t angle = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                         ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
        Motor_SetAbsAngle(feedback,
                          (float)angle * LK_ANGLE_DEGREE_PER_LSB * MOTOR_DEG_TO_RAD -
                          (float)motor->base.zero_point * MOTOR_TWO_PI /
                          (float)LK_ENCODER_COUNTS);
        break;
    }
    default:
        return;
    }
    feedback->timestamp++;
}

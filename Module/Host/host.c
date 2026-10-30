#include "host.h"

#include <string.h>

#include "bsp_dwt.h"

#define HOST_CRC8_POLY              0x07U
#define HOST_CRC8_INIT              0x00U
#define HOST_CRC16_POLY             0x1021U
#define HOST_CRC16_INIT             0xFFFFU
#define HOST_CRC_MSB8               0x80U
#define HOST_CRC_MSB16              0x8000U
#define HOST_BITS_PER_BYTE          8U
#define HOST_BYTE_MASK              0xFFU
#define HOST_HALF_WORD_MASK         0xFFFFU
#define HOST_HALF_WORD_SHIFT        8U
#define HOST_WORD_SHIFT             16U
#define HOST_HEADER_CRC_SPAN        8U      /* 帧头 CRC-8 覆盖前 8 字节 */

#define HOST_RAD_TO_DEG             57.29577951f
#define HOST_TEMP_MAX               127.0f
#define HOST_TEMP_MIN               (-128.0f)
#define HOST_CMD_AGE_NONE           0xFFFFU /* 还没有上位机力矩命令 */

/* 帧头各字段偏移 */
#define HOST_OFFSET_SOF0            0U
#define HOST_OFFSET_SOF1            1U
#define HOST_OFFSET_VERSION         2U
#define HOST_OFFSET_TYPE            3U
#define HOST_OFFSET_SEQUENCE        4U
#define HOST_OFFSET_LENGTH          6U
#define HOST_OFFSET_FLAGS           7U
#define HOST_OFFSET_HEADER_CRC      8U

/* JOINT_STATE 负载头各字段偏移（相对负载起点） */
#define HOST_HEAD_OFFSET_TIME       0U
#define HOST_HEAD_OFFSET_ARM_STATE  4U
#define HOST_HEAD_OFFSET_COUNT      5U
#define HOST_HEAD_OFFSET_REC_SIZE   6U
#define HOST_HEAD_OFFSET_RESERVED   7U
#define HOST_HEAD_OFFSET_FLAGS      8U
#define HOST_HEAD_OFFSET_CMD_AGE    10U

/* 单个关节记录各字段偏移（相对记录起点） */
#define HOST_REC_OFFSET_ANGLE       0U
#define HOST_REC_OFFSET_VELOCITY    4U
#define HOST_REC_OFFSET_TORQUE      8U
#define HOST_REC_OFFSET_TARGET      12U
#define HOST_REC_OFFSET_TORQUE_CMD  16U
#define HOST_REC_OFFSET_ENCODER     20U
#define HOST_REC_OFFSET_STATUS      22U
#define HOST_REC_OFFSET_ERROR_ID    23U
#define HOST_REC_OFFSET_TEMP        24U
#define HOST_REC_OFFSET_FLAGS       25U
#define HOST_REC_OFFSET_FEEDBACK    26U

/*--------------------------------------校验--------------------------------------*/
/* CRC-8/SMBUS：poly 0x07 init 0x00，check("123456789") = 0xF4 */
uint8_t Host_Crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = HOST_CRC8_INIT;

    for (uint16_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < HOST_BITS_PER_BYTE; ++bit)
        {
            crc = (crc & HOST_CRC_MSB8) != 0U ? (uint8_t)((uint32_t)(crc << 1) ^ HOST_CRC8_POLY) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* CRC-16/CCITT-FALSE：poly 0x1021 init 0xFFFF，check("123456789") = 0x29B1 */
uint16_t Host_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = HOST_CRC16_INIT;

    for (uint16_t i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << HOST_HALF_WORD_SHIFT);
        for (uint8_t bit = 0U; bit < HOST_BITS_PER_BYTE; ++bit)
        {
            crc = (crc & HOST_CRC_MSB16) != 0U ? (uint16_t)((uint32_t)(crc << 1) ^ HOST_CRC16_POLY) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/*--------------------------------------小端写入--------------------------------------*/
/* 逐字节写，不依赖结构体打包与对齐 */
static void Host_PutU16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value & HOST_BYTE_MASK);
    out[1] = (uint8_t)(value >> HOST_HALF_WORD_SHIFT);
}

static void Host_PutU32(uint8_t *out, uint32_t value)
{
    Host_PutU16(out, (uint16_t)(value & HOST_HALF_WORD_MASK));
    Host_PutU16(out + sizeof(uint16_t), (uint16_t)(value >> HOST_WORD_SHIFT));
}

static void Host_PutFloat(uint8_t *out, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    Host_PutU32(out, bits);
}

static int8_t Host_ClampTemperature(float temperature)
{
    if (temperature > HOST_TEMP_MAX)
    {
        return (int8_t)HOST_TEMP_MAX;
    }
    if (temperature < HOST_TEMP_MIN)
    {
        return (int8_t)HOST_TEMP_MIN;
    }
    return (int8_t)temperature;
}

/*--------------------------------------打包--------------------------------------*/
/* 关节 joint 的一条记录。角速度、力矩来自电机口径的反馈，按关节方向折算成关节口径，和角度同一个正方向 */
static void Host_PackJointRecord(const Arm_t *arm, uint8_t joint, uint8_t *out)
{
    const Feedback_t *feedback = &arm->config.motor.joint[joint]->feedback;
    /* 方向没填（0）按 +1，与 Arm_UpdateJointAngle 一致 */
    const float direction = arm->config.param.direction[joint] < 0 ? -1.0f : 1.0f;
    const uint8_t online = feedback->status != MOTOR_NO_FEEDBACK ? HOST_JOINT_FLAG_ONLINE : 0U;

    Host_PutFloat(out + HOST_REC_OFFSET_ANGLE, arm->observer.joint_deg[joint]);
    Host_PutFloat(out + HOST_REC_OFFSET_VELOCITY, feedback->omega_rads * HOST_RAD_TO_DEG * direction);
    Host_PutFloat(out + HOST_REC_OFFSET_TORQUE, feedback->torque_nm * direction);
    Host_PutFloat(out + HOST_REC_OFFSET_TARGET, arm->joint_ctrl[joint].target_deg);
    Host_PutFloat(out + HOST_REC_OFFSET_TORQUE_CMD, arm->joint_ctrl[joint].output);
    Host_PutU16(out + HOST_REC_OFFSET_ENCODER, feedback->encoder);
    out[HOST_REC_OFFSET_STATUS] = (uint8_t)feedback->status;
    out[HOST_REC_OFFSET_ERROR_ID] = feedback->error_id;
    out[HOST_REC_OFFSET_TEMP] = (uint8_t)Host_ClampTemperature(feedback->temp);
    out[HOST_REC_OFFSET_FLAGS] = online;
    Host_PutU16(out + HOST_REC_OFFSET_FEEDBACK, (uint16_t)(feedback->timestamp & HOST_HALF_WORD_MASK));
}

static void Host_PackJointState(const Arm_t *arm, uint8_t *payload)
{
    uint16_t flags = 0U;

    if (arm->ctrl_align_pending != 0U)
    {
        flags |= HOST_FLAG_CTRL_ALIGNING;
    }

    Host_PutU32(payload + HOST_HEAD_OFFSET_TIME, (uint32_t)DWT_GetTimeline_us());
    payload[HOST_HEAD_OFFSET_ARM_STATE] = (uint8_t)arm->state;
    payload[HOST_HEAD_OFFSET_COUNT] = ARM_JOINT_COUNT;
    payload[HOST_HEAD_OFFSET_REC_SIZE] = HOST_JOINT_RECORD_SIZE;
    payload[HOST_HEAD_OFFSET_RESERVED] = 0U;
    Host_PutU16(payload + HOST_HEAD_OFFSET_FLAGS, flags);
    Host_PutU16(payload + HOST_HEAD_OFFSET_CMD_AGE, HOST_CMD_AGE_NONE);   /* 目前不接收命令，一律"从未收到" */

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Host_PackJointRecord(arm, i, payload + HOST_JOINT_STATE_HEAD_SIZE + i * HOST_JOINT_RECORD_SIZE);
    }
}

/* 在 host->frame 里按协议打一个整帧，返回帧长 */
static uint16_t Host_PackFrame(Host_t *host, uint8_t type)
{
    uint8_t *frame = host->frame;
    uint16_t crc16;

    frame[HOST_OFFSET_SOF0] = HOST_FRAME_SOF0;
    frame[HOST_OFFSET_SOF1] = HOST_FRAME_SOF1;
    frame[HOST_OFFSET_VERSION] = HOST_FRAME_VERSION;
    frame[HOST_OFFSET_TYPE] = type;
    Host_PutU16(frame + HOST_OFFSET_SEQUENCE, host->sequence);
    frame[HOST_OFFSET_LENGTH] = (uint8_t)HOST_JOINT_STATE_SIZE;
    frame[HOST_OFFSET_FLAGS] = 0U;
    frame[HOST_OFFSET_HEADER_CRC] = Host_Crc8(frame, HOST_HEADER_CRC_SPAN);

    Host_PackJointState(host->arm, frame + HOST_FRAME_HEADER_SIZE);

    crc16 = Host_Crc16(frame, HOST_FRAME_HEADER_SIZE + HOST_JOINT_STATE_SIZE);
    Host_PutU16(frame + HOST_FRAME_HEADER_SIZE + HOST_JOINT_STATE_SIZE, crc16);
    return HOST_JOINT_STATE_FRAME_SIZE;
}

/*--------------------------------------通信--------------------------------------*/
void Host_Init(Host_t *host, const HostConfig_t *config, const Arm_t *arm)
{
    memset(host, 0, sizeof(*host));
    host->config = *config;
    host->arm = arm;
    if (host->config.param.state_divider == 0U)
    {
        host->config.param.state_divider = 1U;
    }
}

void Host_Transmit(Host_t *host, UartInstance_t *uart)
{
    uint16_t length;

    if (++host->divider_count < host->config.param.state_divider)
    {
        return;
    }
    host->divider_count = 0U;

    length = Host_PackFrame(host, HOST_MSG_JOINT_STATE);
    /* 忙就丢这一帧：序号照常递增，上位机能据此统计丢帧 */
    if (UART_TransmitDMAEx(uart, host->frame, length) != HAL_OK)
    {
        host->drop_count++;
    }
    host->sequence++;
}

static void Host_OnUartTransmit(void *context, UartInstance_t *uart)
{
    Host_Transmit((Host_t *)context, uart);
}

BaseType_t Host_BindUart(Host_t *host)
{
    return UART_BindTx(host->config.periph.uart, Host_OnUartTransmit, host);
}

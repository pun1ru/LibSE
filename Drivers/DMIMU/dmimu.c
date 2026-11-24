#include "dmimu.h"

#include <string.h>

#define DMIMU_RS485_HEADER              0xA5U
#define DMIMU_RS485_TAIL                0x5AU
#define DMIMU_RS485_TYPE_CMD            0x0CU
#define DMIMU_RS485_TYPE_DATA           0x0DU
#define DMIMU_RS485_OP_READ             0x00U
#define DMIMU_RS485_OP_WRITE            0x01U
#define DMIMU_RS485_DATA_OFFSET         5U
#define DMIMU_RS485_DATA_LEN            16U
#define DMIMU_RS485_ACK_OFFSET          21U
#define DMIMU_RS485_CRC_OFFSET          22U
#define DMIMU_RS485_CRC_START           1U
#define DMIMU_RS485_CRC_LEN             21U
#define DMIMU_RS485_CMD_MAX             9U

#define DMIMU_CAN_CMD_HEADER            0xCCU
#define DMIMU_CAN_CMD_SEPARATOR         0xDDU
#define DMIMU_CAN_TYPE_ACCEL_TEMP       1U
#define DMIMU_CAN_TYPE_GYRO             2U
#define DMIMU_CAN_TYPE_EULER            3U
#define DMIMU_CAN_TYPE_QUAT             4U
#define DMIMU_CAN_CMD_RESTART           0U
#define DMIMU_CAN_CMD_YAW_ZERO          5U
#define DMIMU_CAN_CMD_BAUD_INDEX        12U
#define DMIMU_CAN_CMD_SAVE              254U
#define DMIMU_CAN_PITCH_HALF_RANGE      90.0f
#define DMIMU_CAN_ROLL_HALF_RANGE       180.0f
#define DMIMU_CAN_QUAT_MAX_RAW          16383U
#define DMIMU_CAN_TEMP_MAX              255U

#define DMIMU_CMD_REBOOT                0U
#define DMIMU_CMD_SAVE                  1U
#define DMIMU_CMD_YAW_ZERO              2U
#define DMIMU_CMD_BAUD_INDEX            7U
#define DMIMU_BAUD_INDEX_MAX            12U
#define DMIMU_VALUE_LIMIT               1.0e4f

/* CRC-8/SAE-J1850，覆盖 485 帧偏移 1~21 */
#define DMIMU_CRC8_POLY                 0x1DU
#define DMIMU_CRC8_INIT                 0xFFU
#define DMIMU_CRC8_XOROUT               0xFFU

static uint32_t DMIMU_NowMs(void)
{
    return (uint32_t)xTaskGetTickCountFromISR() * (uint32_t)portTICK_PERIOD_MS;
}

static uint8_t DMIMU_Crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = DMIMU_CRC8_INIT;
    for (uint16_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x80U) != 0U ? (uint8_t)((crc << 1) ^ DMIMU_CRC8_POLY)
                                      : (uint8_t)(crc << 1);
        }
    }
    return (uint8_t)(crc ^ DMIMU_CRC8_XOROUT);
}

static uint16_t DMIMU_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8);
}

static uint32_t DMIMU_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static float DMIMU_ReadF32(const uint8_t *data)
{
    uint32_t raw = DMIMU_ReadU32(data);
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static bool DMIMU_ValueSane(float value)
{
    return value == value && value <= DMIMU_VALUE_LIMIT && value >= -DMIMU_VALUE_LIMIT;
}

static float DMIMU_DecodeRange(uint16_t raw, float half)
{
    return (float)raw / 65535.0f * (2.0f * half) - half;
}

/* 14 bit 字段，位流高位在前，bit_start 为 0/14/28/42 */
static uint16_t DMIMU_QuatField(const uint8_t *data, uint16_t bit_start)
{
    uint16_t value = 0U;
    for (uint16_t i = 0U; i < 14U; ++i) {
        uint16_t bit_pos = (uint16_t)(bit_start + i);
        uint8_t bit = (uint8_t)(7U - (bit_pos & 7U));
        if (((data[bit_pos >> 3] >> bit) & 1U) != 0U) {
            value |= (uint16_t)(1U << (13U - i));
        }
    }
    return value;
}

static void DMIMU_Store(DMIMU_t *imu, DMIMUDataType_t type, const float *value,
                        uint8_t count, uint32_t now_ms)
{
    float *target[] = {imu->feedback.acc, imu->feedback.gyro,
                       imu->feedback.euler, imu->feedback.quat};
    memcpy(target[type], value, count * sizeof(float));
    imu->feedback.tick_ms[type] = now_ms;
    imu->feedback.valid |= (uint8_t)DMIMU_VALID_BIT(type);
    imu->stats.frame_ok++;
}

static void DMIMU_FinishTx(DMIMU_t *imu, uint32_t now_ms)
{
    imu->tx_inflight = 0U;
    imu->last_done_ms = now_ms;
}

/* 命令应答只保留最新一笔，没被取走的直接覆盖 */
static void DMIMU_StoreCmdResult(DMIMU_t *imu, uint8_t cmd, uint8_t ack,
                                 uint32_t value)
{
    imu->cmd_result.cmd = cmd;
    imu->cmd_result.ack = ack;
    imu->cmd_result.value = value;
    imu->cmd_result.pending = 1U;
}

static bool DMIMU_SendCan(DMIMU_t *imu, uint16_t id, const uint8_t data[DMIMU_CAN_FRAME_LEN])
{
    return CANTransmit_Enqueue(imu->config.can->can, id, data) == pdPASS;
}

static bool DMIMU_SendRs485(DMIMU_t *imu)
{
    return UART_TransmitDMAEx(imu->config.uart, imu->tx_buf, DMIMU_RS485_FRAME_LEN) == HAL_OK;
}

static void DMIMU_BuildRs485Frame(DMIMU_t *imu, uint8_t data_type, uint8_t id,
                                  uint8_t op, const uint8_t *payload)
{
    memset(imu->tx_buf, 0, DMIMU_RS485_FRAME_LEN);
    imu->tx_buf[0] = DMIMU_RS485_HEADER;
    imu->tx_buf[1] = data_type;
    imu->tx_buf[2] = (uint8_t)imu->config.slave_id;
    imu->tx_buf[3] = id;
    imu->tx_buf[4] = op;
    if (payload != NULL) {
        memcpy(&imu->tx_buf[DMIMU_RS485_DATA_OFFSET], payload, DMIMU_RS485_DATA_LEN);
    }
    imu->tx_buf[DMIMU_RS485_CRC_OFFSET] = DMIMU_Crc8(&imu->tx_buf[DMIMU_RS485_CRC_START],
                                                     DMIMU_RS485_CRC_LEN);
    imu->tx_buf[DMIMU_RS485_FRAME_LEN - 1U] = DMIMU_RS485_TAIL;
}

static bool DMIMU_StartTx(DMIMU_t *imu, uint8_t data_type, uint8_t id, uint8_t op,
                          const uint8_t *payload)
{
    if (imu->tx_inflight != 0U) {
        return false;
    }
    uint32_t now_ms = DMIMU_NowMs();
    if (imu->config.link == DMIMU_LINK_CAN) {
        uint8_t frame[DMIMU_CAN_FRAME_LEN] = {
            DMIMU_CAN_CMD_HEADER, id, op, DMIMU_CAN_CMD_SEPARATOR,
            payload != NULL ? payload[0] : 0U,
            payload != NULL ? payload[1] : 0U,
            payload != NULL ? payload[2] : 0U,
            payload != NULL ? payload[3] : 0U
        };
        if (!DMIMU_SendCan(imu, imu->config.slave_id, frame)) {
            return false;
        }
    } else {
        DMIMU_BuildRs485Frame(imu, data_type, id, op, payload);
        if (!DMIMU_SendRs485(imu)) {
            return false;
        }
    }
    imu->tx_inflight = 1U;
    imu->tx_is_cmd = data_type == DMIMU_RS485_TYPE_CMD ? 1U : 0U;
    imu->tx_id = id;
    imu->tx_start_ms = now_ms;
    return true;
}

static bool DMIMU_RequestData(DMIMU_t *imu, DMIMUDataType_t type)
{
    uint8_t id = imu->config.link == DMIMU_LINK_CAN ? (uint8_t)((uint8_t)type + 1U)
                                                    : (uint8_t)type;
    return DMIMU_StartTx(imu, DMIMU_RS485_TYPE_DATA, id, DMIMU_RS485_OP_READ, NULL);
}

static bool DMIMU_SendCmd(DMIMU_t *imu, uint8_t cmd, uint8_t op, uint32_t value)
{
    uint8_t payload[4] = {
        (uint8_t)value, (uint8_t)(value >> 8),
        (uint8_t)(value >> 16), (uint8_t)(value >> 24)
    };
    return DMIMU_StartTx(imu, DMIMU_RS485_TYPE_CMD, cmd, op, payload);
}

/* 485 帧重组：错位时滑到下一个帧头 */
static void DMIMU_FeedRs485(DMIMU_t *imu, const uint8_t *data, uint16_t size)
{
    uint32_t now_ms = DMIMU_NowMs();
    for (uint16_t i = 0U; i < size; ++i) {
        if (imu->rx_len == 0U && data[i] != DMIMU_RS485_HEADER) {
            continue;
        }
        imu->rx_buf[imu->rx_len++] = data[i];
        if (imu->rx_len < DMIMU_RS485_FRAME_LEN) {
            continue;
        }
        imu->rx_len = 0U;
        const uint8_t *frame = imu->rx_buf;
        if (frame[DMIMU_RS485_FRAME_LEN - 1U] != DMIMU_RS485_TAIL ||
            frame[2] != (uint8_t)imu->config.slave_id) {
            imu->stats.format_error++;
            continue;
        }
        if (frame[DMIMU_RS485_CRC_OFFSET] !=
            DMIMU_Crc8(&frame[DMIMU_RS485_CRC_START], DMIMU_RS485_CRC_LEN)) {
            imu->stats.crc_error++;
            continue;
        }
        uint8_t matched = imu->tx_inflight != 0U && frame[1] == (imu->tx_is_cmd != 0U ?
                          DMIMU_RS485_TYPE_CMD : DMIMU_RS485_TYPE_DATA) &&
                          frame[3] == imu->tx_id && frame[4] == DMIMU_RS485_OP_READ;
        uint8_t ack = frame[DMIMU_RS485_ACK_OFFSET];
        if (frame[1] == DMIMU_RS485_TYPE_DATA && frame[4] == DMIMU_RS485_OP_READ &&
            ack == DMIMU_ACK_SUCCESS && frame[3] < (uint8_t)DMIMU_DATA_COUNT) {
            uint8_t count = frame[3] == (uint8_t)DMIMU_DATA_QUAT ? 4U : 3U;
            float value[4];
            bool sane = true;
            for (uint8_t axis = 0U; axis < count; ++axis) {
                value[axis] = DMIMU_ReadF32(&frame[DMIMU_RS485_DATA_OFFSET + axis * 4U]);
                sane = sane && DMIMU_ValueSane(value[axis]);
            }
            if (sane) {
                DMIMU_Store(imu, (DMIMUDataType_t)frame[3], value, count, now_ms);
            } else {
                imu->stats.format_error++;
            }
        } else if (frame[1] == DMIMU_RS485_TYPE_CMD) {
            DMIMU_StoreCmdResult(imu, frame[3], ack,
                                 DMIMU_ReadU32(&frame[DMIMU_RS485_DATA_OFFSET]));
            imu->stats.frame_ok++;
        } else {
            imu->stats.format_error++;
        }
        if (matched) {
            DMIMU_FinishTx(imu, now_ms);
        }
    }
}

static void DMIMU_OnUartRx(void *context, const uint8_t *data, uint16_t size)
{
    DMIMU_FeedRs485((DMIMU_t *)context, data, size);
}

static void DMIMU_OnUartError(void *context)
{
    ((DMIMU_t *)context)->rx_len = 0U;
}

static void DMIMU_FeedCan(DMIMU_t *imu, const uint8_t data[DMIMU_CAN_FRAME_LEN])
{
    uint32_t now_ms = DMIMU_NowMs();
    if (data[0] == DMIMU_CAN_CMD_HEADER && data[2] == DMIMU_CAN_CMD_SEPARATOR) {
        DMIMU_StoreCmdResult(imu, data[1], data[3], DMIMU_ReadU32(&data[4]));
        if (imu->tx_inflight != 0U && imu->tx_is_cmd != 0U && data[1] == imu->tx_id) {
            DMIMU_FinishTx(imu, now_ms);
        }
        imu->stats.frame_ok++;
        return;
    }
    if (data[0] < DMIMU_CAN_TYPE_ACCEL_TEMP || data[0] > DMIMU_CAN_TYPE_QUAT) {
        imu->stats.format_error++;
        return;
    }
    DMIMUDataType_t type = (DMIMUDataType_t)(data[0] - 1U);
    float value[4];
    uint8_t count = 3U;
    switch (type) {
    case DMIMU_DATA_ACCEL:
        value[0] = DMIMU_DecodeRange(DMIMU_ReadU16(&data[2]), imu->config.accel_half_range);
        value[1] = DMIMU_DecodeRange(DMIMU_ReadU16(&data[4]), imu->config.accel_half_range);
        value[2] = DMIMU_DecodeRange(DMIMU_ReadU16(&data[6]), imu->config.accel_half_range);
        imu->feedback.temp = (float)data[1];
        break;
    case DMIMU_DATA_GYRO:
        value[0] = DMIMU_DecodeRange(DMIMU_ReadU16(&data[2]), imu->config.gyro_half_range);
        value[1] = DMIMU_DecodeRange(DMIMU_ReadU16(&data[4]), imu->config.gyro_half_range);
        value[2] = DMIMU_DecodeRange(DMIMU_ReadU16(&data[6]), imu->config.gyro_half_range);
        break;
    case DMIMU_DATA_EULER:
        value[0] = DMIMU_DecodeRange(DMIMU_ReadU16(&data[6]), DMIMU_CAN_ROLL_HALF_RANGE);
        value[1] = DMIMU_DecodeRange(DMIMU_ReadU16(&data[2]), DMIMU_CAN_PITCH_HALF_RANGE);
        value[2] = DMIMU_DecodeRange(DMIMU_ReadU16(&data[4]), DMIMU_CAN_ROLL_HALF_RANGE);
        break;
    case DMIMU_DATA_QUAT:
        count = 4U;
        for (uint8_t i = 0U; i < 4U; ++i) {
            value[i] = (float)DMIMU_QuatField(&data[1], (uint16_t)(i * 14U)) /
                       (float)DMIMU_CAN_QUAT_MAX_RAW * 2.0f - 1.0f;
        }
        break;
    default:
        return;
    }
    bool sane = true;
    for (uint8_t i = 0U; i < count; ++i) {
        sane = sane && DMIMU_ValueSane(value[i]);
    }
    if (!sane) {
        imu->stats.format_error++;
        return;
    }
    DMIMU_Store(imu, type, value, count, now_ms);
    /* 读姿态命令 1~4 的应答是数据帧本身，没有命令应答帧 */
    if (imu->tx_inflight != 0U && imu->tx_is_cmd == 0U && imu->tx_id == data[0]) {
        DMIMU_FinishTx(imu, now_ms);
    }
}

static void DMIMU_OnCanRx(void *context, uint16_t id, const uint8_t *data, uint8_t length)
{
    DMIMU_t *imu = (DMIMU_t *)context;
    if (id != imu->config.master_id || length != DMIMU_CAN_FRAME_LEN) {
        return;
    }
    DMIMU_FeedCan(imu, data);
}

void DMIMU_ConfigDefault(DMIMUConfig_t *config)
{
    memset(config, 0, sizeof(*config));
    config->link = DMIMU_LINK_CAN;
    config->slave_id = DMIMU_SLAVE_ID_DEFAULT;
    config->master_id = DMIMU_MASTER_ID_DEFAULT;
    config->timeout_ms = DMIMU_TIMEOUT_MS_DEFAULT;
    config->request_gap_ms = DMIMU_REQUEST_GAP_MS_DEFAULT;
    config->offline_ms = DMIMU_OFFLINE_MS_DEFAULT;
    config->auto_request = true;
    config->accel_half_range = DMIMU_ACCEL_HALF_RANGE_DEFAULT;
    config->gyro_half_range = DMIMU_GYRO_HALF_RANGE_DEFAULT;
}

bool DMIMU_Init(DMIMU_t *imu, const DMIMUConfig_t *config)
{
    memset(imu, 0, sizeof(*imu));
    imu->config = *config;
    if (config->link == DMIMU_LINK_CAN) {
        return CAN_BindRx(config->can, config->master_id, DMIMU_OnCanRx, imu) == pdPASS;
    }
    return UART_BindRx(config->uart, DMIMU_OnUartRx, DMIMU_OnUartError, imu) == pdPASS;
}

void DMIMU_Update(DMIMU_t *imu)
{
    uint32_t now_ms = DMIMU_NowMs();
    if (imu->tx_inflight != 0U) {
        if ((uint32_t)(now_ms - imu->tx_start_ms) >= imu->config.timeout_ms) {
            imu->tx_inflight = 0U;
            imu->last_done_ms = now_ms;
            imu->stats.timeout_count++;
        }
        return;
    }
    if (imu->config.auto_request &&
        (uint32_t)(now_ms - imu->last_done_ms) >= imu->config.request_gap_ms &&
        DMIMU_RequestData(imu, (DMIMUDataType_t)imu->next_type)) {
        imu->next_type = (uint8_t)((imu->next_type + 1U) % (uint8_t)DMIMU_DATA_COUNT);
    }
}

bool DMIMU_Request(DMIMU_t *imu, DMIMUDataType_t type)
{
    return DMIMU_RequestData(imu, type);
}

bool DMIMU_IsOnline(const DMIMU_t *imu)
{
    uint32_t newest = 0U;
    bool any = false;
    for (uint8_t i = 0U; i < (uint8_t)DMIMU_DATA_COUNT; ++i) {
        if ((imu->feedback.valid & (uint8_t)DMIMU_VALID_BIT(i)) != 0U &&
            (!any || (int32_t)(imu->feedback.tick_ms[i] - newest) > 0)) {
            newest = imu->feedback.tick_ms[i];
            any = true;
        }
    }
    return any && (uint32_t)(DMIMU_NowMs() - newest) <= imu->config.offline_ms;
}

bool DMIMU_ReadReg(DMIMU_t *imu, uint8_t reg)
{
    return DMIMU_SendCmd(imu, reg, DMIMU_RS485_OP_READ, 0U);
}

bool DMIMU_WriteReg(DMIMU_t *imu, uint8_t reg, uint32_t value)
{
    return DMIMU_SendCmd(imu, reg, DMIMU_RS485_OP_WRITE, value);
}

bool DMIMU_SetSettingMode(DMIMU_t *imu, bool enable)
{
    return DMIMU_WriteReg(imu, DMIMU_REG_MODE, enable ? DMIMU_MODE_SETTING : DMIMU_MODE_NORMAL);
}

bool DMIMU_SetBaudIndex(DMIMU_t *imu, uint8_t index)
{
    if (index > DMIMU_BAUD_INDEX_MAX) {
        return false;
    }
    uint8_t cmd = imu->config.link == DMIMU_LINK_CAN ? DMIMU_CAN_CMD_BAUD_INDEX
                                                     : DMIMU_CMD_BAUD_INDEX;
    return DMIMU_SendCmd(imu, cmd, DMIMU_RS485_OP_WRITE, index);
}

bool DMIMU_YawZero(DMIMU_t *imu)
{
    uint8_t cmd = imu->config.link == DMIMU_LINK_CAN ? DMIMU_CAN_CMD_YAW_ZERO
                                                     : DMIMU_CMD_YAW_ZERO;
    return DMIMU_SendCmd(imu, cmd, DMIMU_RS485_OP_WRITE, 0U);
}

bool DMIMU_Save(DMIMU_t *imu)
{
    uint8_t cmd = imu->config.link == DMIMU_LINK_CAN ? DMIMU_CAN_CMD_SAVE : DMIMU_CMD_SAVE;
    return DMIMU_SendCmd(imu, cmd, DMIMU_RS485_OP_WRITE, 0U);
}

bool DMIMU_Restart(DMIMU_t *imu)
{
    uint8_t cmd = imu->config.link == DMIMU_LINK_CAN ? DMIMU_CAN_CMD_RESTART : DMIMU_CMD_REBOOT;
    return DMIMU_SendCmd(imu, cmd, DMIMU_RS485_OP_WRITE, 0U);
}

bool DMIMU_GetCmdResult(DMIMU_t *imu, DMIMUCmdResult_t *result)
{
    if (imu->cmd_result.pending == 0U) {
        return false;
    }
    *result = imu->cmd_result;
    result->pending = 0U;
    imu->cmd_result.pending = 0U;
    return true;
}

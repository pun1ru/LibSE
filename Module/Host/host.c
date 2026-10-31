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
#define HOST_HEADER_CRC_SPAN        8U

#define HOST_RAD_TO_DEG             57.29577951f
#define HOST_TEMP_MAX               127.0f
#define HOST_TEMP_MIN               (-128.0f)
#define HOST_MILLISECONDS_PER_SECOND 1000U
#define HOST_TASK_PERIOD_MS         2U

#define HOST_OFFSET_SOF0            0U
#define HOST_OFFSET_SOF1            1U
#define HOST_OFFSET_VERSION         2U
#define HOST_OFFSET_TYPE            3U
#define HOST_OFFSET_SEQUENCE        4U
#define HOST_OFFSET_LENGTH          6U
#define HOST_OFFSET_FLAGS           7U
#define HOST_OFFSET_HEADER_CRC      8U

#define HOST_HEAD_OFFSET_TIME       0U
#define HOST_HEAD_OFFSET_ARM_STATE  4U
#define HOST_HEAD_OFFSET_COUNT      5U
#define HOST_HEAD_OFFSET_REC_SIZE   6U
#define HOST_HEAD_OFFSET_RESERVED   7U
#define HOST_HEAD_OFFSET_FLAGS      8U
#define HOST_HEAD_OFFSET_CMD_AGE    10U

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

#define HOST_ACK_RESULT_OK                 0U
#define HOST_ACK_RESULT_BAD_PARAM          3U
#define HOST_ACK_RESULT_BAD_LENGTH         4U
#define HOST_ACK_RESULT_UNSUPPORTED        5U
#define HOST_PARAM_TYPE_U8                 0U
#define HOST_PARAM_TYPE_I8                 1U
#define HOST_PARAM_TYPE_U16                2U
#define HOST_PARAM_TYPE_F32                6U
#define HOST_FRAME_FLAG_ACK_REQ            0x01U

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

static uint16_t Host_GetU16(const uint8_t *in)
{
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << HOST_HALF_WORD_SHIFT));
}

static void Host_PutFloat(uint8_t *out, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    Host_PutU32(out, bits);
}

static float Host_GetFloat(const uint8_t *in)
{
    uint32_t bits = (uint32_t)in[0] |
                    ((uint32_t)in[1] << 8U) |
                    ((uint32_t)in[2] << 16U) |
                    ((uint32_t)in[3] << 24U);
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
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

static uint32_t Host_NowMs(void)
{
    return (uint32_t)(DWT_GetTimeline_us() / HOST_MILLISECONDS_PER_SECOND);
}

static uint32_t Host_ElapsedMs(uint32_t now_ms, uint32_t since_ms)
{
    return (uint32_t)(now_ms - since_ms);
}

static uint16_t Host_PackPayloadFrame(Host_t *host, uint8_t type, const uint8_t *payload,
                                      uint16_t length)
{
    uint8_t *frame = host->frame;
    uint16_t crc16;

    if (length > HOST_FRAME_MAX_PAYLOAD)
    {
        return 0U;
    }
    frame[HOST_OFFSET_SOF0] = HOST_FRAME_SOF0;
    frame[HOST_OFFSET_SOF1] = HOST_FRAME_SOF1;
    frame[HOST_OFFSET_VERSION] = HOST_FRAME_VERSION;
    frame[HOST_OFFSET_TYPE] = type;
    Host_PutU16(frame + HOST_OFFSET_SEQUENCE, host->sequence);
    frame[HOST_OFFSET_LENGTH] = (uint8_t)length;
    frame[HOST_OFFSET_FLAGS] = 0U;
    frame[HOST_OFFSET_HEADER_CRC] = Host_Crc8(frame, HOST_HEADER_CRC_SPAN);
    memcpy(frame + HOST_FRAME_HEADER_SIZE, payload, length);
    crc16 = Host_Crc16(frame, (uint16_t)(HOST_FRAME_HEADER_SIZE + length));
    Host_PutU16(frame + HOST_FRAME_HEADER_SIZE + length, crc16);
    return (uint16_t)(HOST_FRAME_OVERHEAD + length);
}

static void Host_PackJointRecord(const Arm_t *arm, uint8_t joint, uint8_t *out)
{
    const Feedback_t *feedback = &arm->config.motor.joint[joint]->feedback;
    const float direction = arm->config.param.direction[joint] < 0 ? -1.0f : 1.0f;
    const uint8_t bit = (uint8_t)(1U << joint);
    uint8_t flags = feedback->status != MOTOR_NO_FEEDBACK ? HOST_JOINT_FLAG_ONLINE : 0U;

    if (arm->state == ARM_HOST && (arm->host_target_valid_mask & bit) != 0U)
    {
        flags |= HOST_JOINT_FLAG_HOST_TARGET_VALID;
    }
    if (arm->state == ARM_HOST && (arm->host_closed_loop_mask & bit) != 0U)
    {
        flags |= HOST_JOINT_FLAG_HOST_CLOSED_LOOP;
    }

    Host_PutFloat(out + HOST_REC_OFFSET_ANGLE, arm->observer.joint_deg[joint]);
    Host_PutFloat(out + HOST_REC_OFFSET_VELOCITY, feedback->omega_rads * HOST_RAD_TO_DEG * direction);
    Host_PutFloat(out + HOST_REC_OFFSET_TORQUE, feedback->torque_nm * direction);
    Host_PutFloat(out + HOST_REC_OFFSET_TARGET, arm->joint_ctrl[joint].target_deg);
    Host_PutFloat(out + HOST_REC_OFFSET_TORQUE_CMD, arm->joint_ctrl[joint].output);
    Host_PutU16(out + HOST_REC_OFFSET_ENCODER, feedback->encoder);
    out[HOST_REC_OFFSET_STATUS] = (uint8_t)feedback->status;
    out[HOST_REC_OFFSET_ERROR_ID] = feedback->error_id;
    out[HOST_REC_OFFSET_TEMP] = (uint8_t)Host_ClampTemperature(feedback->temp);
    out[HOST_REC_OFFSET_FLAGS] = flags;
    Host_PutU16(out + HOST_REC_OFFSET_FEEDBACK, (uint16_t)(feedback->timestamp & HOST_HALF_WORD_MASK));
}

static void Host_PackJointState(Host_t *host, uint8_t *payload)
{
    const Arm_t *arm = host->arm;
    const uint32_t now_ms = Host_NowMs();
    uint16_t flags = 0U;
    uint16_t age = HOST_CMD_AGE_NONE;

    if (arm->rc_online != 0U)
    {
        flags |= HOST_FLAG_RC_ONLINE;
    }
    if (host->ever_rx != 0U && Host_ElapsedMs(now_ms, host->last_rx_ms) <= HOST_LINK_TIMEOUT_MS)
    {
        flags |= HOST_FLAG_HOST_LINK_OK;
    }
    if (arm->ctrl_align_pending != 0U || arm->host_align_pending != 0U)
    {
        flags |= HOST_FLAG_CTRL_ALIGNING;
    }
    if (arm->state == ARM_HOST && arm->host_closed_loop_mask != 0U)
    {
        flags |= HOST_FLAG_HOST_CLOSED_LOOP;
    }
    if (host->ever_cmd != 0U)
    {
        uint32_t age32 = Host_ElapsedMs(now_ms, host->last_cmd_ms);
        age = age32 > HOST_CMD_AGE_NONE ? HOST_CMD_AGE_NONE : (uint16_t)age32;
    }

    Host_PutU32(payload + HOST_HEAD_OFFSET_TIME, (uint32_t)DWT_GetTimeline_us());
    payload[HOST_HEAD_OFFSET_ARM_STATE] = (uint8_t)arm->state;
    payload[HOST_HEAD_OFFSET_COUNT] = ARM_JOINT_COUNT;
    payload[HOST_HEAD_OFFSET_REC_SIZE] = HOST_JOINT_RECORD_SIZE;
    payload[HOST_HEAD_OFFSET_RESERVED] = 0U;
    Host_PutU16(payload + HOST_HEAD_OFFSET_FLAGS, flags);
    Host_PutU16(payload + HOST_HEAD_OFFSET_CMD_AGE, age);

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Host_PackJointRecord(arm, i, payload + HOST_JOINT_STATE_HEAD_SIZE + i * HOST_JOINT_RECORD_SIZE);
    }
}

static void Host_DropRxPrefix(Host_t *host, uint16_t count)
{
    if (count >= host->rx_length)
    {
        host->rx_length = 0U;
        return;
    }
    memmove(host->rx_buffer, host->rx_buffer + count, (size_t)(host->rx_length - count));
    host->rx_length = (uint16_t)(host->rx_length - count);
}

static void Host_EnqueueRequest(Host_t *host, const uint8_t *frame, uint16_t total)
{
    const uint8_t length = frame[HOST_OFFSET_LENGTH];
    HostRequest_t *request;
    UBaseType_t saved;
    uint16_t next;

    (void)total;

    saved = taskENTER_CRITICAL_FROM_ISR();
    next = (uint16_t)((host->rx_head + 1U) % HOST_RX_QUEUE_SIZE);
    if (next == host->rx_tail)
    {
        host->rx_drop_count++;
        taskEXIT_CRITICAL_FROM_ISR(saved);
        return;
    }

    request = &host->requests[host->rx_head];
    request->seq = Host_GetU16(frame + HOST_OFFSET_SEQUENCE);
    request->type = frame[HOST_OFFSET_TYPE];
    request->flags = frame[HOST_OFFSET_FLAGS];
    request->length = length <= HOST_MAX_REQUEST_PAYLOAD ? length : 0U;
    request->overflow = length <= HOST_MAX_REQUEST_PAYLOAD ? 0U : 1U;
    if (request->length != 0U)
    {
        memcpy(request->payload, frame + HOST_FRAME_HEADER_SIZE, request->length);
    }
    host->rx_head = next;
    taskEXIT_CRITICAL_FROM_ISR(saved);
}

static void Host_ParseRxBuffer(Host_t *host)
{
    for (;;)
    {
        uint16_t start = 0U;
        uint8_t length;
        uint16_t total;
        uint16_t received_crc;

        while (start + 1U < host->rx_length &&
               !(host->rx_buffer[start] == HOST_FRAME_SOF0 && host->rx_buffer[start + 1U] == HOST_FRAME_SOF1))
        {
            ++start;
        }
        if (start != 0U)
        {
            Host_DropRxPrefix(host, start);
        }
        if (host->rx_length < 2U)
        {
            return;
        }
        if (host->rx_buffer[0] != HOST_FRAME_SOF0 || host->rx_buffer[1] != HOST_FRAME_SOF1)
        {
            if (host->rx_buffer[host->rx_length - 1U] == HOST_FRAME_SOF0)
            {
                Host_DropRxPrefix(host, (uint16_t)(host->rx_length - 1U));
            }
            else
            {
                host->rx_length = 0U;
            }
            return;
        }
        if (host->rx_length < HOST_FRAME_HEADER_SIZE)
        {
            return;
        }
        if (Host_Crc8(host->rx_buffer, HOST_HEADER_CRC_SPAN) != host->rx_buffer[HOST_OFFSET_HEADER_CRC])
        {
            Host_DropRxPrefix(host, 1U);
            continue;
        }

        length = host->rx_buffer[HOST_OFFSET_LENGTH];
        if (length > HOST_FRAME_MAX_PAYLOAD)
        {
            Host_DropRxPrefix(host, 1U);
            continue;
        }
        total = (uint16_t)(HOST_FRAME_OVERHEAD + length);
        if (host->rx_length < total)
        {
            return;
        }
        received_crc = Host_GetU16(host->rx_buffer + total - HOST_FRAME_TRAILER_SIZE);
        if (Host_Crc16(host->rx_buffer, (uint16_t)(total - HOST_FRAME_TRAILER_SIZE)) != received_crc)
        {
            Host_DropRxPrefix(host, 1U);
            continue;
        }
        if ((host->rx_buffer[HOST_OFFSET_VERSION] >> 4U) != (HOST_FRAME_VERSION >> 4U))
        {
            Host_DropRxPrefix(host, total);
            continue;
        }

        host->last_rx_ms = Host_NowMs();
        host->ever_rx = 1U;
        Host_EnqueueRequest(host, host->rx_buffer, total);
        Host_DropRxPrefix(host, total);
    }
}

static void Host_OnUartReceive(void *context, const uint8_t *data, uint16_t size)
{
    Host_t *host = (Host_t *)context;

    if (size >= HOST_RX_BUFFER_SIZE)
    {
        memcpy(host->rx_buffer, data + size - HOST_RX_BUFFER_SIZE, HOST_RX_BUFFER_SIZE);
        host->rx_length = HOST_RX_BUFFER_SIZE;
        Host_ParseRxBuffer(host);
        return;
    }
    if ((uint16_t)(host->rx_length + size) > HOST_RX_BUFFER_SIZE)
    {
        Host_DropRxPrefix(host, (uint16_t)(host->rx_length + size - HOST_RX_BUFFER_SIZE));
    }
    memcpy(host->rx_buffer + host->rx_length, data, size);
    host->rx_length = (uint16_t)(host->rx_length + size);
    Host_ParseRxBuffer(host);
}

static void Host_OnUartError(void *context)
{
    Host_t *host = (Host_t *)context;
    host->rx_length = 0U;
}

static bool Host_PopRequest(Host_t *host, HostRequest_t *request)
{
    bool available;

    taskENTER_CRITICAL();
    available = host->rx_tail != host->rx_head;

    if (available)
    {
        *request = host->requests[host->rx_tail];
        host->rx_tail = (uint16_t)((host->rx_tail + 1U) % HOST_RX_QUEUE_SIZE);
    }
    taskEXIT_CRITICAL();
    return available;
}

static bool Host_PushResponse(Host_t *host, uint8_t type, const uint8_t *payload, uint8_t length)
{
    HostResponse_t *response;
    uint16_t next = (uint16_t)((host->tx_head + 1U) % HOST_TX_QUEUE_SIZE);

    if (length > HOST_DEVICE_INFO_SIZE || next == host->tx_tail)
    {
        host->tx_drop_count++;
        return false;
    }
    response = &host->responses[host->tx_head];
    response->type = type;
    response->length = length;
    memcpy(response->payload, payload, length);
    host->tx_head = next;
    return true;
}

static HostReplyCache_t *Host_FindReply(Host_t *host, uint8_t type, uint16_t seq)
{
    for (uint8_t i = 0U; i < host->cache_count; ++i)
    {
        if (host->replies[i].seq == seq && host->replies[i].type == type)
        {
            return &host->replies[i];
        }
    }
    return NULL;
}

static void Host_StoreReply(Host_t *host, uint8_t type, uint16_t seq,
                            const uint8_t *payload, uint8_t length)
{
    HostReplyCache_t *entry = &host->replies[host->cache_head];

    entry->seq = seq;
    entry->type = type;
    entry->length = length;
    memcpy(entry->payload, payload, length);
    host->cache_head = (uint16_t)((host->cache_head + 1U) % HOST_REPLY_CACHE_SIZE);
    if (host->cache_count < HOST_REPLY_CACHE_SIZE)
    {
        host->cache_count++;
    }
}

static void Host_QueueReply(Host_t *host, const HostRequest_t *request, uint8_t type,
                            const uint8_t *payload, uint8_t length)
{
    (void)Host_PushResponse(host, type, payload, length);
    Host_StoreReply(host, request->type, request->seq, payload, length);
}

static void Host_MakeAck(Host_t *host, const HostRequest_t *request, uint8_t result, uint32_t detail)
{
    uint8_t payload[HOST_ACK_SIZE];

    Host_PutU16(payload, request->seq);
    payload[2] = request->type;
    payload[3] = result;
    Host_PutU32(payload + 4U, detail);
    Host_QueueReply(host, request, HOST_MSG_ACK, payload, HOST_ACK_SIZE);
}

static bool Host_ParamInfo(uint16_t param_id, uint8_t *type, uint8_t *count, uint8_t *access)
{
    *access = 1U;
    switch (param_id)
    {
    case HOST_PARAM_STATE_RATE_HZ:
    case HOST_PARAM_STATUS_RATE_HZ:
    case HOST_PARAM_LINK_TIMEOUT:
    case HOST_PARAM_TORQUE_HOLD_MAX:
        *type = HOST_PARAM_TYPE_U16;
        *count = 1U;
        return true;
    case HOST_PARAM_ZERO_POINT:
        *type = HOST_PARAM_TYPE_U16;
        *count = ARM_JOINT_COUNT;
        return true;
    case HOST_PARAM_DIRECTION:
        *type = HOST_PARAM_TYPE_I8;
        *count = ARM_JOINT_COUNT;
        return true;
    case HOST_PARAM_LIMIT_MIN:
    case HOST_PARAM_LIMIT_MAX:
    case HOST_PARAM_GAIN_KP:
    case HOST_PARAM_GAIN_KI:
    case HOST_PARAM_GAIN_KD:
    case HOST_PARAM_GAIN_D_ALPHA:
    case HOST_PARAM_GAIN_SUM_MAX:
    case HOST_PARAM_GAIN_OUTPUT_MAX:
    case HOST_PARAM_HOST_TORQUE_LIMIT:
        *type = HOST_PARAM_TYPE_F32;
        *count = ARM_JOINT_COUNT;
        return true;
    case HOST_PARAM_DH:
        *type = HOST_PARAM_TYPE_F32;
        *count = ARM_JOINT_COUNT * 4U;
        return true;
    default:
        *type = 0U;
        *count = 0U;
        *access = 0U;
        return false;
    }
}

static bool Host_ParamValue(const Host_t *host, uint16_t param_id, uint8_t index, uint8_t *type,
                            uint8_t *value)
{
    const Arm_t *arm = host->arm;

    memset(value, 0, 4U);
    switch (param_id)
    {
    case HOST_PARAM_STATE_RATE_HZ:
    {
        uint16_t rate = (uint16_t)(HOST_MILLISECONDS_PER_SECOND /
                                  (HOST_TASK_PERIOD_MS * host->config.param.state_divider));
        Host_PutU16(value, rate);
        break;
    }
    case HOST_PARAM_STATUS_RATE_HZ:
        Host_PutU16(value, 0U);
        break;
    case HOST_PARAM_LINK_TIMEOUT:
        Host_PutU16(value, HOST_LINK_TIMEOUT_MS);
        break;
    case HOST_PARAM_TORQUE_HOLD_MAX:
        Host_PutU16(value, 0U);
        break;
    case HOST_PARAM_ZERO_POINT:
        Host_PutU16(value, arm->config.param.zero_point[index]);
        break;
    case HOST_PARAM_DIRECTION:
        value[0] = (uint8_t)(int8_t)arm->config.param.direction[index];
        break;
    case HOST_PARAM_LIMIT_MIN:
        Host_PutFloat(value, arm->config.param.limit[index].min_deg);
        break;
    case HOST_PARAM_LIMIT_MAX:
        Host_PutFloat(value, arm->config.param.limit[index].max_deg);
        break;
    case HOST_PARAM_GAIN_KP:
        Host_PutFloat(value, arm->config.param.gain[index].kp);
        break;
    case HOST_PARAM_GAIN_KI:
        Host_PutFloat(value, arm->config.param.gain[index].ki);
        break;
    case HOST_PARAM_GAIN_KD:
        Host_PutFloat(value, arm->config.param.gain[index].kd);
        break;
    case HOST_PARAM_GAIN_D_ALPHA:
        Host_PutFloat(value, arm->config.param.gain[index].d_alpha);
        break;
    case HOST_PARAM_GAIN_SUM_MAX:
        Host_PutFloat(value, arm->config.param.gain[index].sum_error_max);
        break;
    case HOST_PARAM_GAIN_OUTPUT_MAX:
        Host_PutFloat(value, arm->config.param.gain[index].output_max);
        break;
    case HOST_PARAM_HOST_TORQUE_LIMIT:
        Host_PutFloat(value, 0.0f);
        break;
    case HOST_PARAM_DH:
    {
        const uint8_t row = (uint8_t)(index / 4U);
        const uint8_t col = (uint8_t)(index % 4U);
        float dh_value = 0.0f;

        if (col == 0U)
        {
            dh_value = arm->config.param.dh[row].alpha_deg;
        }
        else if (col == 1U)
        {
            dh_value = arm->config.param.dh[row].a_m;
        }
        else if (col == 2U)
        {
            dh_value = arm->config.param.dh[row].d_m;
        }
        else
        {
            dh_value = arm->config.param.dh[row].theta_offset_deg;
        }
        Host_PutFloat(value, dh_value);
        break;
    }
    default:
        return false;
    }
    (void)type;
    return true;
}

static void Host_ProcessParamRead(Host_t *host, const HostRequest_t *request)
{
    uint8_t payload[HOST_PARAM_VALUE_SIZE];
    uint8_t type = 0U;
    uint8_t count = 0U;
    uint8_t access = 0U;
    uint8_t value[4] = {0U, 0U, 0U, 0U};
    uint8_t result = HOST_ACK_RESULT_BAD_PARAM;
    uint16_t param_id = 0U;
    uint8_t index = 0U;

    if (request->overflow != 0U || request->length < 3U)
    {
        result = HOST_ACK_RESULT_BAD_LENGTH;
    }
    else
    {
        param_id = Host_GetU16(request->payload);
        index = request->payload[2];
        if (Host_ParamInfo(param_id, &type, &count, &access) && index < count)
        {
            if (Host_ParamValue(host, param_id, index, &type, value))
            {
                result = HOST_ACK_RESULT_OK;
            }
            else
            {
                access = 0U;
            }
        }
        else if (access == 0U)
        {
            result = HOST_ACK_RESULT_UNSUPPORTED;
        }
    }

    Host_PutU16(payload, request->seq);
    Host_PutU16(payload + 2U, param_id);
    payload[4] = index;
    payload[5] = type;
    payload[6] = result;
    payload[7] = access;
    memcpy(payload + 8U, value, sizeof(value));
    Host_QueueReply(host, request, HOST_MSG_PARAM_VALUE, payload, HOST_PARAM_VALUE_SIZE);
}

static void Host_ProcessSetTarget(Host_t *host, const HostRequest_t *request)
{
    uint8_t action;
    uint8_t joint;
    float target_deg;
    bool accepted = false;

    if (request->overflow != 0U || request->length != HOST_SET_HOST_TARGET_SIZE)
    {
        if ((request->flags & HOST_FRAME_FLAG_ACK_REQ) != 0U)
        {
            Host_MakeAck(host, request, HOST_ACK_RESULT_BAD_LENGTH, 0U);
        }
        return;
    }

    joint = request->payload[0];
    action = request->payload[1];
    target_deg = Host_GetFloat(request->payload + 4U);
    if (action == HOST_TARGET_ACTION_STOP_ALL)
    {
        Arm_StopHostControl(host->arm);
        accepted = true;
    }
    else if (action == HOST_TARGET_ACTION_SET_OPEN || action == HOST_TARGET_ACTION_SET_CLOSED)
    {
        if (joint < ARM_JOINT_COUNT)
        {
            accepted = Arm_SetHostTarget(host->arm, joint, target_deg,
                                         action == HOST_TARGET_ACTION_SET_CLOSED) == ARM_HOST_TARGET_OK;
        }
    }
    else
    {
        if ((request->flags & HOST_FRAME_FLAG_ACK_REQ) != 0U)
        {
            Host_MakeAck(host, request, HOST_ACK_RESULT_BAD_PARAM, 0U);
        }
        return;
    }

    if (accepted)
    {
        host->last_cmd_ms = Host_NowMs();
        host->ever_cmd = 1U;
    }
    if ((request->flags & HOST_FRAME_FLAG_ACK_REQ) != 0U)
    {
        Host_MakeAck(host, request,
                     accepted ? HOST_ACK_RESULT_OK : HOST_ACK_RESULT_BAD_PARAM, 0U);
    }
}

static void Host_ProcessRequest(Host_t *host, const HostRequest_t *request)
{
    HostReplyCache_t *cached = Host_FindReply(host, request->type, request->seq);
    uint8_t payload[HOST_DEVICE_INFO_SIZE] = {0U};

    if (cached != NULL)
    {
        (void)Host_PushResponse(host, cached->type, cached->payload, cached->length);
        return;
    }

    switch (request->type)
    {
    case HOST_MSG_HEARTBEAT:
        break;
    case HOST_MSG_DEVICE_INFO_REQ:
        payload[0] = HOST_FRAME_VERSION;
        payload[1] = ARM_JOINT_COUNT;
        payload[2] = HOST_JOINT_RECORD_SIZE;
        payload[3] = (uint8_t)host->arm->config.state.role;
        Host_PutU16(payload + 4U, HOST_FRAME_MAX_SIZE);
        Host_PutU16(payload + 6U, (uint16_t)(HOST_MILLISECONDS_PER_SECOND /
                                             (HOST_TASK_PERIOD_MS * host->config.param.state_divider)));
        Host_PutU16(payload + 8U, 0U);
        Host_PutU16(payload + 10U, HOST_LINK_TIMEOUT_MS);
        Host_PutU16(payload + 12U, 0U);
        Host_PutU16(payload + 14U, 1U);
        memcpy(payload + 16U, "7axisArm", 8U);
        Host_QueueReply(host, request, HOST_MSG_DEVICE_INFO, payload, HOST_DEVICE_INFO_SIZE);
        break;
    case HOST_MSG_PARAM_READ:
        Host_ProcessParamRead(host, request);
        break;
    case HOST_MSG_SET_HOST_TARGET:
        Host_ProcessSetTarget(host, request);
        break;
    default:
        if ((request->flags & HOST_FRAME_FLAG_ACK_REQ) != 0U)
        {
            Host_MakeAck(host, request, HOST_ACK_RESULT_UNSUPPORTED, 0U);
        }
        break;
    }
}

static void Host_ProcessCommands(Host_t *host)
{
    HostRequest_t request;
    const uint32_t now_ms = Host_NowMs();

    while (Host_PopRequest(host, &request))
    {
        Host_ProcessRequest(host, &request);
    }

    if (host->arm->state == ARM_HOST && host->arm->host_closed_loop_mask != 0U &&
        (host->ever_rx == 0U || Host_ElapsedMs(now_ms, host->last_rx_ms) > HOST_LINK_TIMEOUT_MS))
    {
        Arm_StopHostControl(host->arm);
    }
}

void Host_Init(Host_t *host, const HostConfig_t *config, Arm_t *arm)
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

    Host_ProcessCommands(host);

    if (host->tx_tail != host->tx_head)
    {
        HostResponse_t *response = &host->responses[host->tx_tail];

        length = Host_PackPayloadFrame(host, response->type, response->payload, response->length);
        if (length != 0U && UART_TransmitDMAEx(uart, host->frame, length) == HAL_OK)
        {
            host->tx_tail = (uint16_t)((host->tx_tail + 1U) % HOST_TX_QUEUE_SIZE);
            host->sequence++;
        }
        return;
    }

    if (++host->divider_count < host->config.param.state_divider)
    {
        return;
    }
    host->divider_count = 0U;

    Host_PackJointState(host, host->joint_payload);
    length = Host_PackPayloadFrame(host, HOST_MSG_JOINT_STATE, host->joint_payload, HOST_JOINT_STATE_SIZE);
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
    BaseType_t tx = UART_BindTx(host->config.periph.uart, Host_OnUartTransmit, host);
    BaseType_t rx = UART_BindRx(host->config.periph.uart, Host_OnUartReceive, Host_OnUartError, host);

    return (tx == pdPASS && rx == pdPASS) ? pdPASS : pdFAIL;
}

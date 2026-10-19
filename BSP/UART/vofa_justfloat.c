#include "vofa_justfloat.h"

#include <string.h>

/* 帧尾字节，小端读为 float +Inf */
static const uint8_t vofa_tail[VOFA_JUSTFLOAT_TAIL_SIZE] = {0x00U, 0x00U, 0x80U, 0x7FU};

/* 按小端把 float 的位模式写进 dst，不依赖主机字节序，也不要求 dst 对齐 */
static void VofaJustFloat_PutFloat(uint8_t *dst, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    dst[0] = (uint8_t)(bits & 0xFFU);
    dst[1] = (uint8_t)((bits >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((bits >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((bits >> 24U) & 0xFFU);
}

/* 在最后一个通道后面补帧尾 */
static void VofaJustFloat_WriteTail(VofaJustFloat_t *frame)
{
    size_t offset = (size_t)frame->channel_count * VOFA_JUSTFLOAT_CHANNEL_SIZE;

    memcpy(&frame->buffer[offset], vofa_tail, VOFA_JUSTFLOAT_TAIL_SIZE);
}

bool VofaJustFloat_Init(VofaJustFloat_t *frame, uint8_t *buffer, size_t capacity)
{
    if (frame == NULL)
    {
        return false;
    }

    frame->buffer = NULL;
    frame->capacity = 0U;
    frame->channel_count = 0U;
    if (buffer == NULL ||
        capacity < (VOFA_JUSTFLOAT_CHANNEL_SIZE + VOFA_JUSTFLOAT_TAIL_SIZE))
    {
        return false;
    }

    frame->buffer = buffer;
    frame->capacity = capacity;
    VofaJustFloat_WriteTail(frame);
    return true;
}

void VofaJustFloat_Reset(VofaJustFloat_t *frame)
{
    if (frame == NULL || frame->buffer == NULL)
    {
        return;
    }

    frame->channel_count = 0U;
    VofaJustFloat_WriteTail(frame);
}

uint16_t VofaJustFloat_FreeChannels(const VofaJustFloat_t *frame)
{
    if (frame == NULL || frame->buffer == NULL)
    {
        return 0U;
    }

    /* 帧尾占的 4 字节先扣掉，剩下的才能放通道 */
    size_t usable = frame->capacity - VOFA_JUSTFLOAT_TAIL_SIZE;
    size_t max_channels = usable / VOFA_JUSTFLOAT_CHANNEL_SIZE;
    if (max_channels > UINT16_MAX)
    {
        max_channels = UINT16_MAX;
    }
    if (max_channels <= frame->channel_count)
    {
        return 0U;
    }
    return (uint16_t)(max_channels - frame->channel_count);
}

bool VofaJustFloat_AddFloats(VofaJustFloat_t *frame, const float *values, uint16_t count)
{
    if (frame == NULL || frame->buffer == NULL || (values == NULL && count != 0U))
    {
        return false;
    }
    if (count > VofaJustFloat_FreeChannels(frame))
    {
        return false;
    }

    size_t offset = (size_t)frame->channel_count * VOFA_JUSTFLOAT_CHANNEL_SIZE;
    for (uint16_t i = 0U; i < count; ++i)
    {
        VofaJustFloat_PutFloat(&frame->buffer[offset], values[i]);
        offset += VOFA_JUSTFLOAT_CHANNEL_SIZE;
    }
    frame->channel_count = (uint16_t)(frame->channel_count + count);
    VofaJustFloat_WriteTail(frame);
    return true;
}

bool VofaJustFloat_AddFloat(VofaJustFloat_t *frame, float value)
{
    return VofaJustFloat_AddFloats(frame, &value, 1U);
}

const uint8_t *VofaJustFloat_Data(const VofaJustFloat_t *frame)
{
    if (frame == NULL || frame->buffer == NULL)
    {
        return NULL;
    }
    return frame->buffer;
}

size_t VofaJustFloat_Size(const VofaJustFloat_t *frame)
{
    if (frame == NULL || frame->buffer == NULL || frame->channel_count == 0U)
    {
        return 0U;
    }
    return ((size_t)frame->channel_count * VOFA_JUSTFLOAT_CHANNEL_SIZE) +
           VOFA_JUSTFLOAT_TAIL_SIZE;
}

uint16_t VofaJustFloat_ChannelCount(const VofaJustFloat_t *frame)
{
    if (frame == NULL)
    {
        return 0U;
    }
    return frame->channel_count;
}

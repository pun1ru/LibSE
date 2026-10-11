#include "referee.h"

#include <string.h>

void Referee_Init(Referee_t *referee)
{
    if (referee == NULL) {
        return;
    }

    memset(referee, 0, sizeof(*referee));
}

void Referee_Reset(Referee_t *referee)
{
    if (referee == NULL) {
        return;
    }

    referee->frame_size = 0U;
}

void Referee_Input(Referee_t *referee, const uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0U; i < size; ++i) {
        uint8_t byte = data[i];
        if (referee->frame_size == 0U && byte != 0xA5U) {
            continue;
        }
        if (referee->frame_size >= REFEREE_BUFFER_SIZE) {
            referee->frame_size = 0U;
            if (byte != 0xA5U) {
                continue;
            }
        }
        referee->frame_buffer[referee->frame_size++] = byte;
        if (referee->frame_size < 5U) {
            continue;
        }
        if (Verify_CRC8_Check_Sum(referee->frame_buffer, 5U) == 0U) {
            memmove(referee->frame_buffer, referee->frame_buffer + 1U,
                    --referee->frame_size);
            continue;
        }

        uint16_t payload_size = (uint16_t)referee->frame_buffer[1] |
                                ((uint16_t)referee->frame_buffer[2] << 8);
        uint32_t frame_size = (uint32_t)payload_size + 9U;
        if (frame_size > REFEREE_BUFFER_SIZE) {
            referee->frame_size = 0U;
            continue;
        }
        if (referee->frame_size == frame_size) {
            RefereeReceive(frame_size, referee->frame_buffer);
            referee->frame_size = 0U;
        }
    }
}

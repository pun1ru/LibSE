#ifndef VOFA_JUSTFLOAT_H
#define VOFA_JUSTFLOAT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* VOFA+ JustFloat 协议：一帧 = N 个小端 float32 通道 + 4 字节帧尾 00 00 80 7F
 * 帧尾按小端读是 float +Inf，上位机用它对齐帧边界
 * 本文件只负责往调用者给的缓冲区里拼帧，不碰串口也不依赖 OS，发送见 vofa_uart.h */
#define VOFA_JUSTFLOAT_TAIL_SIZE     4U
#define VOFA_JUSTFLOAT_CHANNEL_SIZE  4U

typedef struct {
    uint8_t *buffer;        ///< 调用者提供的帧缓冲区
    size_t capacity;        ///< 缓冲区总字节数，含帧尾
    uint16_t channel_count; ///< 已加入的通道数
} VofaJustFloat_t;

/* 绑定缓冲区并清空；capacity 小于一个通道加帧尾时返回 false，实例保持不可用 */
bool VofaJustFloat_Init(VofaJustFloat_t *frame, uint8_t *buffer, size_t capacity);

/* 清空通道，开始拼下一帧 */
void VofaJustFloat_Reset(VofaJustFloat_t *frame);

/* 追加一个通道，帧尾自动跟在最后一个通道后面
 * 缓冲区放不下时返回 false，帧内容保持不变 */
bool VofaJustFloat_AddFloat(VofaJustFloat_t *frame, float value);

/* 一次追加 count 个通道，要么全部加入要么都不加 */
bool VofaJustFloat_AddFloats(VofaJustFloat_t *frame, const float *values, uint16_t count);

/* 帧数据与长度，可直接交给串口发送
 * 没有通道时长度为 0，避免把只有帧尾的空帧发出去 */
const uint8_t *VofaJustFloat_Data(const VofaJustFloat_t *frame);
size_t VofaJustFloat_Size(const VofaJustFloat_t *frame);

uint16_t VofaJustFloat_ChannelCount(const VofaJustFloat_t *frame);
/* 当前缓冲区最多还能再加多少个通道 */
uint16_t VofaJustFloat_FreeChannels(const VofaJustFloat_t *frame);

#ifdef __cplusplus
}
#endif

#endif

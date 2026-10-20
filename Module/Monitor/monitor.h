#ifndef MONITOR_H
#define MONITOR_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

/* 任务心跳监控：各任务每个周期调 Monitor_Beat，监控任务按固定采样周期取心跳并累计，
 * 窗口结束时与期望心跳数对比，判断任务是否卡死或周期混乱
 * 心跳走 task notification：通知值的第 n 位代表 n 号任务（当前内核不支持带索引的通知），
 * 所以一个采样周期内同一任务的多次心跳只算一次，任务周期短于采样周期时按采样周期算期望 */
#define MONITOR_TASK_MAX    16U     /* 通知值有 32 位，这里留 16 个任务够用 */

/*--------------------------------------通信层--------------------------------------*/
typedef struct {
    TaskHandle_t *monitor_task;     /* 接收心跳的监控任务句柄变量的地址，任务创建后才有值，创建前心跳被忽略 */
} MonitorComm_t;

/*--------------------------------------参数层--------------------------------------*/
typedef struct {
    uint16_t sample_period_ms;          /* Monitor_Collect 的调用周期 */
    uint8_t task_count;
    const uint16_t *task_period_ms;     /* 各任务的运行周期，下标即任务号 */
} MonitorParam_t;

typedef struct {
    MonitorComm_t comm;
    MonitorParam_t param;
} MonitorConfig_t;

/*--------------------------------------运行状态--------------------------------------*/
typedef struct {
    uint16_t beat_count;        /* 当前窗口内收到的心跳数 */
    uint32_t offline_count;     /* 累计判离线的窗口数 */
    uint8_t online;
} MonitorTaskStatus_t;

typedef struct {
    MonitorConfig_t config;
    MonitorTaskStatus_t task[MONITOR_TASK_MAX];
    uint16_t sample_count;      /* 当前窗口内 Collect 的次数 */
} Monitor_t;

void Monitor_Init(Monitor_t *monitor, const MonitorConfig_t *config);

/* 任务周期末尾调用，task_id 即 task_period_ms 的下标 */
void Monitor_Beat(const Monitor_t *monitor, uint8_t task_id);

/* 监控任务每个采样周期调一次：取走并清零通知值，累计各任务心跳 */
void Monitor_Collect(Monitor_t *monitor);

/* 监控任务每个窗口调一次：判断各任务是否在线，清零窗口计数；有任务刚从在线变成离线返回 1 */
uint8_t Monitor_Evaluate(Monitor_t *monitor);

#endif

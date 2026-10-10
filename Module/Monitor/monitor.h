#ifndef MONITOR_H
#define MONITOR_H

#include <stdint.h>

#define MONITOR_TASK_COUNT 3U
#define MONITOR_OFFLINE_SAMPLE_LIMIT 5U

typedef enum {
    MONITOR_TASK_RECEIVE = 0U,
    MONITOR_TASK_TRANSMIT = 1U,
    MONITOR_TASK_INS = 2U
} MonitorTaskId_t;

typedef struct {
    uint32_t heartbeat;
    uint32_t error_count;
    uint32_t missed_samples;
    uint8_t ready;
    uint8_t online;
} MonitorTaskStatus_t;

typedef struct {
    uint32_t sample_count;
    MonitorTaskStatus_t task[MONITOR_TASK_COUNT];
} MonitorStatus_t;

void Monitor_Init(void);
void Monitor_TaskBeat(MonitorTaskId_t task);
void Monitor_TaskError(MonitorTaskId_t task);
void Monitor_SetReady(MonitorTaskId_t task, uint8_t ready);
void Monitor_Sample(void);
void Monitor_GetSnapshot(MonitorStatus_t *status);

#endif

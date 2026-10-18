#ifndef MONITOR_H
#define MONITOR_H

#include <stdint.h>

#define MONITOR_TASK_COUNT 4U

typedef enum {
    MONITOR_TASK_RECEIVE = 0U,
    MONITOR_TASK_TRANSMIT = 1U,
    MONITOR_TASK_INS = 2U,
    MONITOR_TASK_GIMBAL = 3U
} MonitorTaskId_t;

typedef struct {
    uint32_t error_count;
    uint8_t online;
} MonitorTaskStatus_t;

typedef struct {
    MonitorTaskStatus_t task[MONITOR_TASK_COUNT];
} MonitorStatus_t;

void Monitor_Init(void);
void Monitor_TaskBeat(MonitorTaskId_t task);
void Monitor_TaskError(MonitorTaskId_t task);
uint8_t Monitor_Sample(void);
void Monitor_GetSnapshot(MonitorStatus_t *status);

#endif

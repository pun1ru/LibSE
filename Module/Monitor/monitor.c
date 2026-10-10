#include "monitor.h"

#include <stddef.h>
#include <string.h>

static MonitorStatus_t monitor_status;
static uint32_t previous_heartbeat[MONITOR_TASK_COUNT];

static uint8_t Monitor_TaskIdValid(MonitorTaskId_t task)
{
    return (task < MONITOR_TASK_COUNT) ? 1U : 0U;
}

void Monitor_Init(void)
{
    memset(&monitor_status, 0, sizeof(monitor_status));
    memset(previous_heartbeat, 0, sizeof(previous_heartbeat));
}

void Monitor_TaskBeat(MonitorTaskId_t task)
{
    if (Monitor_TaskIdValid(task) == 0U) {
        return;
    }

    ++monitor_status.task[task].heartbeat;
    monitor_status.task[task].ready = 1U;
}

void Monitor_TaskError(MonitorTaskId_t task)
{
    if (Monitor_TaskIdValid(task) == 0U) {
        return;
    }

    ++monitor_status.task[task].error_count;
}

void Monitor_SetReady(MonitorTaskId_t task, uint8_t ready)
{
    if (Monitor_TaskIdValid(task) == 0U) {
        return;
    }

    monitor_status.task[task].ready = (ready != 0U) ? 1U : 0U;
    if (ready == 0U) {
        monitor_status.task[task].online = 0U;
    }
}

void Monitor_Sample(void)
{
    uint8_t task_index;

    ++monitor_status.sample_count;
    for (task_index = 0U; task_index < MONITOR_TASK_COUNT; ++task_index) {
        MonitorTaskStatus_t *task = &monitor_status.task[task_index];

        if (task->heartbeat != previous_heartbeat[task_index]) {
            previous_heartbeat[task_index] = task->heartbeat;
            task->missed_samples = 0U;
            task->online = task->ready;
        } else if (task->missed_samples < UINT32_MAX) {
            ++task->missed_samples;
            if (task->missed_samples >= MONITOR_OFFLINE_SAMPLE_LIMIT) {
                task->online = 0U;
            }
        }

        if (task->ready == 0U) {
            task->online = 0U;
        }
    }
}

void Monitor_GetSnapshot(MonitorStatus_t *status)
{
    if (status == NULL) {
        return;
    }

    memcpy(status, &monitor_status, sizeof(*status));
}

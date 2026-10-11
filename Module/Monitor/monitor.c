#include "monitor.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

static MonitorStatus_t monitor_status;
static uint32_t heartbeat[MONITOR_TASK_COUNT];
static uint32_t previous_heartbeat[MONITOR_TASK_COUNT];

static uint8_t Monitor_TaskIdValid(MonitorTaskId_t task)
{
    return (task < MONITOR_TASK_COUNT) ? 1U : 0U;
}

void Monitor_Init(void)
{
    memset(&monitor_status, 0, sizeof(monitor_status))  ;
    memset(heartbeat, 0, sizeof(heartbeat));
    memset(previous_heartbeat, 0, sizeof(previous_heartbeat));
}

void Monitor_TaskBeat(MonitorTaskId_t task)
{
    if (Monitor_TaskIdValid(task) == 0U) {
        return;
    }

    taskENTER_CRITICAL();
    ++heartbeat[task];
    taskEXIT_CRITICAL();
}

void Monitor_TaskError(MonitorTaskId_t task)
{
    if (Monitor_TaskIdValid(task) == 0U) {
        return;
    }

    taskENTER_CRITICAL();
    ++monitor_status.task[task].error_count;
    taskEXIT_CRITICAL();
}

uint8_t Monitor_Sample(void)
{
    uint8_t task_index;
    uint8_t went_offline = 0U;

    taskENTER_CRITICAL();
    for (task_index = 0U; task_index < MONITOR_TASK_COUNT; ++task_index) {
        MonitorTaskStatus_t *task = &monitor_status.task[task_index];

        if (heartbeat[task_index] == previous_heartbeat[task_index]) {
            if (task->online != 0U) {
                went_offline = 1U;
            }
            task->online = 0U;
            ++task->error_count;
        } else {
            previous_heartbeat[task_index] = heartbeat[task_index];
            task->online = 1U;
        }
    }
    taskEXIT_CRITICAL();

    return went_offline;
}

void Monitor_GetSnapshot(MonitorStatus_t *status)
{
    if (status == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    memcpy(status, &monitor_status, sizeof(*status));
    taskEXIT_CRITICAL();
}

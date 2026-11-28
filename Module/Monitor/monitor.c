#include "monitor.h"

#include <stddef.h>
#include <string.h>

#define MONITOR_NOTIFY_CLEAR_ALL    0xFFFFFFFFU

static void Monitor_DispatchEvent(Monitor_t *monitor, uint8_t task_id,
                                  MonitorEvent_t event)
{
    const MonitorTaskConfig_t *config = &monitor->config.param.tasks[task_id];

    if (config->on_event != NULL) {
        config->on_event(event, config, &monitor->task[task_id],
                         config->context);
    }
}

BaseType_t Monitor_Init(Monitor_t *monitor, const MonitorConfig_t *config)
{
    TickType_t now;

    if (monitor == NULL || config == NULL ||
        config->comm.monitor_task == NULL ||
        config->param.task_count > MONITOR_TASK_MAX ||
        (config->param.task_count != 0U && config->param.tasks == NULL)) {
        return pdFAIL;
    }

    for (uint8_t i = 0U; i < config->param.task_count; ++i) {
        const MonitorTaskConfig_t *task = &config->param.tasks[i];

        if (task->name == NULL || task->task_handle == NULL ||
            task->period_ms == 0U || task->timeout_ms == 0U ||
            task->heartbeat == NULL) {
            return pdFAIL;
        }
    }

    memset(monitor, 0, sizeof(*monitor));
    monitor->config = *config;
    now = xTaskGetTickCount();

    for (uint8_t i = 0U; i < monitor->config.param.task_count; ++i) {
        const MonitorTaskConfig_t *task = &monitor->config.param.tasks[i];

        monitor->task[i].online = 1U;
        monitor->task[i].last_heartbeat_tick = now;
        task->heartbeat->monitor = monitor;
        task->heartbeat->task_id = i;
    }

    return pdPASS;
}

void Monitor_Heartbeat(MonitorHeartbeat_t heartbeat)
{
    Monitor_t *monitor = heartbeat.monitor;
    TaskHandle_t target;

    if (monitor == NULL ||
        heartbeat.task_id >= monitor->config.param.task_count) {
        return;
    }

    monitor->task[heartbeat.task_id].last_heartbeat_tick = xTaskGetTickCount();
    target = *monitor->config.comm.monitor_task;
    if (target != NULL) {
        (void)xTaskNotify(target, 1UL << heartbeat.task_id, eSetBits);
    }
}

void Monitor_Collect(Monitor_t *monitor)
{
    uint32_t pending = 0U;

    if (monitor == NULL) {
        return;
    }

    ++monitor->sample_count;
    if (xTaskNotifyWait(0U, MONITOR_NOTIFY_CLEAR_ALL, &pending, 0U) != pdPASS) {
        return;
    }

    for (uint8_t i = 0U; i < monitor->config.param.task_count; ++i) {
        if ((pending & (1UL << i)) != 0U) {
            ++monitor->task[i].beat_count;
        }
    }
}

uint8_t Monitor_Evaluate(Monitor_t *monitor)
{
    TickType_t now;
    uint8_t lost_count = 0U;

    if (monitor == NULL) {
        return 0U;
    }

    now = xTaskGetTickCount();
    for (uint8_t i = 0U; i < monitor->config.param.task_count; ++i) {
        const MonitorTaskConfig_t *config = &monitor->config.param.tasks[i];
        MonitorTaskStatus_t *status = &monitor->task[i];
        uint32_t silence_ms =
            (uint32_t)(now - status->last_heartbeat_tick) * portTICK_PERIOD_MS;
        uint8_t alive = silence_ms <= config->timeout_ms ? 1U : 0U;

        if (alive != 0U) {
            if (status->online == 0U) {
                status->online = 1U;
                Monitor_DispatchEvent(monitor, i, MONITOR_EVENT_RECOVERED);
            }
        } else {
            ++status->offline_count;
            if (status->online != 0U) {
                status->online = 0U;
                ++lost_count;
                Monitor_DispatchEvent(monitor, i, MONITOR_EVENT_LOST);
            }
        }

        status->beat_count = 0U;
    }
    monitor->sample_count = 0U;
    return lost_count;
}

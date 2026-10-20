#include "monitor.h"

#include <stddef.h>
#include <string.h>

#define MONITOR_ALIVE_RATIO_DIV     2U      /* 心跳数不低于期望的 1/2 才算在线 */
#define MONITOR_NOTIFY_CLEAR_ALL    0xFFFFFFFFU

void Monitor_Init(Monitor_t *monitor, const MonitorConfig_t *config)
{
    memset(monitor, 0, sizeof(*monitor));
    monitor->config = *config;
}

void Monitor_Beat(const Monitor_t *monitor, uint8_t task_id)
{
    TaskHandle_t target = *monitor->config.comm.monitor_task;

    if (target != NULL && task_id < monitor->config.param.task_count)
    {
        (void)xTaskNotify(target, 1UL << task_id, eSetBits);
    }
}

void Monitor_Collect(Monitor_t *monitor)
{
    uint32_t pending = 0U;

    ++monitor->sample_count;
    /* 超时为 0：没有通知就立即返回，不阻塞监控任务 */
    if (xTaskNotifyWait(0U, MONITOR_NOTIFY_CLEAR_ALL, &pending, 0U) != pdPASS)
    {
        return;
    }

    for (uint8_t i = 0U; i < monitor->config.param.task_count; ++i)
    {
        if ((pending & (1UL << i)) != 0U)
        {
            ++monitor->task[i].beat_count;
        }
    }
}

uint8_t Monitor_Evaluate(Monitor_t *monitor)
{
    const MonitorParam_t *param = &monitor->config.param;
    uint32_t elapsed_ms = (uint32_t)monitor->sample_count * param->sample_period_ms;
    uint8_t went_offline = 0U;

    for (uint8_t i = 0U; i < param->task_count; ++i)
    {
        MonitorTaskStatus_t *task = &monitor->task[i];
        /* 任务比采样还快时，一个采样周期最多记一次心跳，期望也按采样周期算 */
        uint32_t beat_period_ms = param->task_period_ms[i] > param->sample_period_ms ?
                                  param->task_period_ms[i] : param->sample_period_ms;
        uint32_t expected = elapsed_ms / beat_period_ms;
        uint8_t alive = (task->beat_count != 0U &&
                         (uint32_t)task->beat_count * MONITOR_ALIVE_RATIO_DIV >= expected) ? 1U : 0U;

        if (alive != 0U)
        {
            task->online = 1U;
        }
        else
        {
            if (task->online != 0U)
            {
                went_offline = 1U;
            }
            task->online = 0U;
            ++task->offline_count;
        }
        task->beat_count = 0U;
    }
    monitor->sample_count = 0U;
    return went_offline;
}

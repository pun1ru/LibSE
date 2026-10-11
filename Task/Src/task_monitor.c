#include "task_monitor.h"
#include "bsp_buzzer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_define.h"

void MonitorTask(void *argument)
{
    TickType_t last_wake;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;) {
        if (Monitor_Sample() != 0U) {
            Buzzer_BeepOnce();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_MONITOR_PERIOD_MS));
    }
}

void MonitorTask_GetSnapshot(MonitorStatus_t *status)
{
    Monitor_GetSnapshot(status);
}

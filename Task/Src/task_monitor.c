#include "task_monitor.h"
#include "task_ins.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_define.h"

void MonitorTask(void *argument)
{
    TickType_t last_wake;
    INS_TaskStatus_t ins_status;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;) {
        INS_Task_GetStatus(&ins_status);
        Monitor_SetReady(MONITOR_TASK_INS, ins_status.ready);
        Monitor_Sample();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_MONITOR_PERIOD_MS));
    }
}

void MonitorTask_GetSnapshot(MonitorStatus_t *status)
{
    Monitor_GetSnapshot(status);
}

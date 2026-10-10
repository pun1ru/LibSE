#include "task_ins.h"
#include "bsp_dwt.h"
#include "FreeRTOS.h"
#include "task_define.h"
#include "task.h"
#include "monitor.h"

#include <stddef.h>

#define INS_MADGWICK_BETA 0.05f
#define INS_RETRY_PERIOD_MS 1000U

INS_t board_ins;
static INS_TaskStatus_t ins_task_status;

void INS_Task(void *argument)
{
    TickType_t last_wake;
    uint32_t last_cycle;
    (void)argument;
    last_wake = xTaskGetTickCount();
    last_cycle = DWT->CYCCNT;

    for (;;) {
        if (board_ins.ready == 0U) {
            ++ins_task_status.init_attempt_count;
            INS_Init(&board_ins, INS_MADGWICK_BETA);
            ins_task_status.ready = board_ins.ready;
            if (board_ins.ready == 0U) {
                ++ins_task_status.init_error_count;
                Monitor_TaskError(MONITOR_TASK_INS);
                Monitor_SetReady(MONITOR_TASK_INS, 0U);
                vTaskDelay(pdMS_TO_TICKS(INS_RETRY_PERIOD_MS));
                last_wake = xTaskGetTickCount();
                continue;
            }

            Monitor_SetReady(MONITOR_TASK_INS, 1U);
            last_wake = xTaskGetTickCount();
            last_cycle = DWT->CYCCNT;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_INS_PERIOD_MS));
        INS_Update(&board_ins, DWT_GetDeltaT(&last_cycle));
        ins_task_status.cycle_count++;
        ins_task_status.update_count = board_ins.update_count;
        ins_task_status.last_dt_s = board_ins.last_dt_s;
        ins_task_status.ready = board_ins.ready;
        Monitor_TaskBeat(MONITOR_TASK_INS);
    }
}

void INS_Task_GetStatus(INS_TaskStatus_t *status)
{
    if (status == NULL) {
        return;
    }

    *status = ins_task_status;
}

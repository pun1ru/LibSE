#include "task_ins.h"
#include "bsp_dwt.h"
#include "FreeRTOS.h"
#include "task_define.h"
#include "task.h"
#include "monitor.h"
#include "task_system.h"

#define INS_MADGWICK_BETA 0.05f
#define INS_RETRY_PERIOD_MS 1000U

void INS_Task(void *argument)
{
    TickType_t last_wake;
    uint32_t last_cycle;
    (void)argument;
    last_wake = xTaskGetTickCount();
    last_cycle = DWT->CYCCNT;

    while(1) {
        if (board_ins.ready == 0U) {
            INS_Init(&board_ins, INS_MADGWICK_BETA);
            last_wake = xTaskGetTickCount();
            last_cycle = DWT->CYCCNT;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_INS_PERIOD_MS));
        INS_Update(&board_ins, DWT_GetDeltaT(&last_cycle));
        Monitor_TaskBeat(MONITOR_TASK_INS);
    }
}

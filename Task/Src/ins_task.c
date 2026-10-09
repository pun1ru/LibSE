#include "ins_task.h"
#include "bsp_dwt.h"
#include "FreeRTOS.h"
#include "task.h"

#define INS_TASK_PERIOD_MS 2U
#define INS_MADGWICK_BETA 0.05f

INS_t board_ins;

void INS_Task(void *argument)
{
    (void)argument;
    DWT_Init(480U);
    INS_Init(&board_ins, INS_MADGWICK_BETA);
    uint32_t last_cycle = DWT->CYCCNT;

    for (;;) {
        if (board_ins.ready != 0U) {
            INS_Update(&board_ins, DWT_GetDeltaT(&last_cycle));
            vTaskDelay(pdMS_TO_TICKS(INS_TASK_PERIOD_MS));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000U));
            INS_Init(&board_ins, INS_MADGWICK_BETA);
            last_cycle = DWT->CYCCNT;
        }
    }
}

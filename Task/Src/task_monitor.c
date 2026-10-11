#include "task_monitor.h"
#include "bsp_buzzer.h"
#include "../../Drivers/WS2812/WS2812.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_define.h"
#include "task_system.h"
static void Task_SendCan1Test(void)
{
    static uint8_t test_data[8] = {
        0x11, 0x22, 0x33, 0x44,
        0x55, 0x66, 0x77, 0x88
    };

    if (CANTransmit_EnqueueLength(&hfdcan1,
                                  0x123U,
                                  test_data,
                                  sizeof(test_data)) != pdPASS) {
        /* CAN1 发送队列满或未初始化 */
        Monitor_TaskError(MONITOR_TASK_TRANSMIT);
    }
}

void MonitorTask(void *argument)
{
    TickType_t last_wake;

    (void)argument;
    last_wake = xTaskGetTickCount();
    WS2812_Breathe_Start(0U, 255U, 0U, 2000U);

    for (;;) {
        Task_SendCan1Test();
        WS2812_Breathe_Update();
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

#include "task_monitor.h"

#include "bsp_buzzer.h"
#include "../../Drivers/WS2812/WS2812.h"
#include "CAN_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_define.h"
#include "task_system.h"

#define TASK_MONITOR_SAMPLE_DIVIDER (TASK_MONITOR_PERIOD_MS / TASK_MOTOR_SEND_PERIOD_MS)
#define TASK_CAN_MONITOR_DIVIDER (TASK_CAN_MONITOR_PERIOD_MS / TASK_MOTOR_SEND_PERIOD_MS)
#define TASK_MONITOR_ALERT_PITCH 69U

void MonitorTask(void *argument)
{
    TickType_t last_wake;
    TickType_t motor_period = pdMS_TO_TICKS(TASK_MOTOR_SEND_PERIOD_MS);
    uint8_t monitor_divider = 0U;
    uint8_t can_monitor_divider = 0U;
    uint8_t buzzer_active = 0U;

    (void)argument;
    last_wake = xTaskGetTickCount();
    WS2812_Breathe_Start(0U, 255U, 0U, 2000U);

    for (;;)
    {
        if (++can_monitor_divider >= TASK_CAN_MONITOR_DIVIDER)
        {
            can_monitor_divider = 0U;
            CAN_MonitorPoll();
        }

        if (++monitor_divider >= TASK_MONITOR_SAMPLE_DIVIDER)
        {
            monitor_divider = 0U;
            WS2812_Breathe_Update();
            if (buzzer_active != 0U)
            {
                Buzzer_PlayPitch(0U);
                buzzer_active = 0U;
            }
            if (Monitor_Sample() != 0U)
            {
                Buzzer_PlayPitch(TASK_MONITOR_ALERT_PITCH);
                buzzer_active = 1U;
            }
        }
        vTaskDelayUntil(&last_wake, motor_period);
        if ((TickType_t)(xTaskGetTickCount() - last_wake) >= motor_period)
        {
            last_wake = xTaskGetTickCount();
        }
    }
}

void MonitorTask_GetSnapshot(MonitorStatus_t *status)
{
    Monitor_GetSnapshot(status);
}

#include "task_monitor.h"

#include "bsp_buzzer.h"
#include "../../Drivers/WS2812/WS2812.h"
#include "CAN_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_define.h"
#include "task_system.h"

#define TASK_FRIC_HIGH_GROUP_LEADER_INDEX 3U
#define TASK_MONITOR_SAMPLE_DIVIDER (TASK_MONITOR_PERIOD_MS / TASK_MOTOR_SEND_PERIOD_MS)
#define TASK_CAN_MONITOR_DIVIDER (TASK_CAN_MONITOR_PERIOD_MS / TASK_MOTOR_SEND_PERIOD_MS)
#define TASK_MONITOR_ALERT_PITCH 69U

static void TaskMonitor_SendZeroMotorFrames(uint8_t lk_phase,
                                            uint16_t dm_enable_phase)
{
    yawMotor->motorcmd.position = 0.0f;
    yawMotor->motorcmd.velocity = 0.0f;
    yawMotor->motorcmd.kp = 0.0f;
    yawMotor->motorcmd.kd = 0.0f;
    yawMotor->motorcmd.torque = 0.0f;
    yawMotor->motorcmd.current = 0.0f;
    if (dm_enable_phase == 0U)
    {
        yawMotor->motorops.enable(yawMotor);
    }
    yawMotor->motorops.sendcommand(yawMotor);

    for (uint8_t index = 0U; index < TASK_FRIC_MOTOR_COUNT; ++index)
    {
        fricMotors[index]->motorcmd.current = 0.0f;
    }
    fricMotors[0]->motorops.sendgroupcommand(fricMotors[0]);
    fricMotors[TASK_FRIC_HIGH_GROUP_LEADER_INDEX]->motorops.sendgroupcommand(
        fricMotors[TASK_FRIC_HIGH_GROUP_LEADER_INDEX]);

    pitchMotor->motorcmd.current = 0.0f;
    if (lk_phase == TASK_LK_REQUEST_DIVIDER - 1U)
    {
        pitchMotor->motorops.requestfeedback(pitchMotor);
    }
    else
    {
        pitchMotor->motorops.sendcommand(pitchMotor);
    }
}

void MonitorTask(void *argument)
{
    TickType_t last_wake;
    TickType_t motor_period = pdMS_TO_TICKS(TASK_MOTOR_SEND_PERIOD_MS);
    uint8_t lk_phase = 0U;
    uint16_t dm_enable_phase = 0U;
    uint8_t monitor_divider = 0U;
    uint8_t can_monitor_divider = 0U;
    uint8_t buzzer_active = 0U;

    (void)argument;
    last_wake = xTaskGetTickCount();
    WS2812_Breathe_Start(0U, 255U, 0U, 2000U);

    for (;;)
    {
        TaskMonitor_SendZeroMotorFrames(lk_phase, dm_enable_phase);
        lk_phase = (uint8_t)((lk_phase + 1U) % TASK_LK_REQUEST_DIVIDER);
        dm_enable_phase = (uint16_t)((dm_enable_phase + 1U) %
                                     TASK_DM_ENABLE_DIVIDER);

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

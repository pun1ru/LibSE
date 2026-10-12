#include "task_transmit.h"

#include "CAN_driver.h"
#include "FreeRTOS.h"
#include "monitor.h"
#include "task.h"
#include "task_define.h"
#include "task_system.h"

#include "CAN_driver.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include "task.h"



void TransmitTask(void *argument)
{
    CanTxFrame_t frame;
    TickType_t last_lk_request;

    (void)argument;
    last_lk_request = xTaskGetTickCount() -
                      pdMS_TO_TICKS(TASK_LK_FEEDBACK_PERIOD_MS);

    while(1) {
        TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - last_lk_request) >=
            pdMS_TO_TICKS(TASK_LK_FEEDBACK_PERIOD_MS))
        {
            pitchMotor->motorops.requestfeedback(pitchMotor);
            last_lk_request = now;
        }
        QueueSetMemberHandle_t ready = xQueueSelectFromSet(
            transmit_can_queues, pdMS_TO_TICKS(TASK_TRANSMIT_PERIOD_MS));
        //有就发    
        while (ready != NULL) {
            if (xQueueReceive(ready, &frame, 0U) == pdPASS) {
                if (fdcanx_send_data(frame.can, frame.id, frame.data,
                                     frame.length) != 0U) {
                    Monitor_TaskError(MONITOR_TASK_TRANSMIT);
                }
            }
            ready = xQueueSelectFromSet(transmit_can_queues, 0U);
        }
        Monitor_TaskBeat(MONITOR_TASK_TRANSMIT);
    }
}

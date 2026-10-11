#include "task_transmit.h"

#include "CAN_driver.h"
#include "FreeRTOS.h"
#include "monitor.h"
#include "task.h"
#include "task_define.h"
#include "task_system.h"

void TransmitTask(void *argument)
{
    CanTxFrame_t frame;

    (void)argument;
    transmit_can_queues = xQueueCreateSet((UBaseType_t)TASK_CAN_INSTANCE_COUNT *
                            (UBaseType_t)CAN_TX_QUEUE_LENGTH);
    for (uint8_t i = 0U; i < TASK_CAN_INSTANCE_COUNT; ++i) {
    xQueueAddToSet(can_instances[i].tx_queue, transmit_can_queues);  
    }

    while(1) {
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

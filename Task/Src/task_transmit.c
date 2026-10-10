#include "task_transmit.h"

#include "CAN_driver.h"
#include "FreeRTOS.h"
#include "monitor.h"
#include "task.h"
#include "task_define.h"
#include "task_system.h"

void TransmitTask(void *argument)
{
    uint8_t instance_count = 0U;
    CanInstance_t *instances = CAN_GetInstances(&instance_count);
    QueueSetHandle_t queues;
    CanTxFrame_t frame;

    (void)argument;
    if (instances == NULL || instance_count == 0U) {
        vTaskDelete(NULL);
        return;
    }

    queues = xQueueCreateSet((UBaseType_t)instance_count *
                            (UBaseType_t)CAN_TX_QUEUE_LENGTH);
    if (queues == NULL) {
        vTaskDelete(NULL);
        return;
    }

    for (uint8_t i = 0U; i < instance_count; ++i) {
        if (instances[i].tx_queue == NULL ||
            xQueueAddToSet(instances[i].tx_queue, queues) != pdPASS) {
            vQueueDelete(queues);
            vTaskDelete(NULL);
            return;
        }
    }

    for (;;) {
        QueueSetMemberHandle_t ready = xQueueSelectFromSet(
            queues, pdMS_TO_TICKS(TASK_TRANSMIT_PERIOD_MS));

        while (ready != NULL) {
            if (xQueueReceive(ready, &frame, 0U) == pdPASS) {
                if (fdcanx_send_data(frame.can, frame.id, frame.data,
                                     frame.length) != 0U) {
                    Monitor_TaskError(MONITOR_TASK_TRANSMIT);
                }
            }
            ready = xQueueSelectFromSet(queues, 0U);
        }
        Monitor_TaskBeat(MONITOR_TASK_TRANSMIT);
    }
}

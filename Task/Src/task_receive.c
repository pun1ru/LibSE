#include "task_receive.h"
#include "CAN_driver.h"
#include "task.h"

void ReceiveTask(void *argument)
{
    CanInstance_t *can = (CanInstance_t *)argument;
    //队列集
    QueueSetHandle_t queues = xQueueCreateSet(3U * CAN_RX_QUEUE_LENGTH);
    if (queues == NULL ||
        xQueueAddToSet(can[0].rx_queue, queues) != pdPASS ||
        xQueueAddToSet(can[1].rx_queue, queues) != pdPASS ||
        xQueueAddToSet(can[2].rx_queue, queues) != pdPASS) {
        vTaskDelete(NULL);
        return;
    }
    CanRxFrame_t frame;
    while(1){
        QueueSetMemberHandle_t ready = xQueueSelectFromSet(queues, portMAX_DELAY);
        if (xQueueReceive(ready, &frame, 0U) != pdPASS) {
            continue;
        }
        if (ready == can[0].rx_queue && can[0].on_receive != NULL) {
            can[0].on_receive(can[0].can, frame.id, frame.data, frame.length);
        } else if (ready == can[1].rx_queue && can[1].on_receive != NULL) {
            can[1].on_receive(can[1].can, frame.id, frame.data, frame.length);
        } else if (ready == can[2].rx_queue && can[2].on_receive != NULL) {
            can[2].on_receive(can[2].can, frame.id, frame.data, frame.length);
        }
    }
}

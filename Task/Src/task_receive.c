#include "task_receive.h"

#include "CAN_driver.h"
#include "FreeRTOS.h"
#include "monitor.h"
#include "referee.h"
#include "remote_control.h"
#include "task.h"
#include "task_define.h"
#include "task_system.h"

#include <stddef.h>
#include <string.h>

#define TASK_UART_INSTANCE_COUNT 6U
#define TASK_UART_PACKET_SIZE BSP_UART_DMA_BUFFER_SIZE

static BSP_UART_t *const task_uart_instances[TASK_UART_INSTANCE_COUNT] = {
    &uart5_instance,
    &uart7_instance,
    &uart1_instance,
    &uart2_instance,
    &uart3_instance,
    &uart10_instance
};

static void TaskReceive_InitProtocols(void)
{
    memset(&referee_receiver, 0, sizeof(referee_receiver));
    RC_Remote_Init(&rc_remote);
}

void TaskReceive_CANCallback(FDCAN_HandleTypeDef *can, uint16_t id,
                             const uint8_t *data, uint8_t length)
{
    (void)can;
    (void)id;
    (void)data;
    (void)length;
}

void CANReceive_Callback(FDCAN_HandleTypeDef *can, uint16_t id,
                         const uint8_t *data, uint8_t length)
{
    TaskReceive_CANCallback(can, id, data, length);
}

void UARTReceive_Callback(BSP_UART_t *uart, const uint8_t *data,
                          uint16_t length)
{
    (void)uart;
    (void)data;
    (void)length;
}

static void TaskReceive_StartUart(void)
{
    for (uint8_t i = 0U; i < TASK_UART_INSTANCE_COUNT; ++i) {
        BSP_UART_t *uart = task_uart_instances[i];

        (void)BSP_UART_SetNotifyTask(uart, xTaskGetCurrentTaskHandle());
        if (BSP_UART_StartReceiveEx(uart) != HAL_OK) {
            Monitor_TaskError(MONITOR_TASK_RECEIVE);
        }
    }
}

static void TaskReceive_ProcessUart(BSP_UART_t *uart)
{
    uint8_t packet[TASK_UART_PACKET_SIZE];
    uint16_t packet_size;

    while (BSP_UART_RxPop(uart, packet, sizeof(packet), &packet_size) == pdTRUE) {
        if (uart == &uart5_instance) {
            RC_Remote_Input(&rc_remote, packet, packet_size);
        } else if (uart == &uart1_instance) {
            Referee_Input(&referee_receiver, packet, packet_size);
        } else {
            UARTReceive_Callback(uart, packet, packet_size);
        }
    }
}

static void TaskReceive_ProcessCan(QueueSetHandle_t queues,
                                   CanInstance_t *instances,
                                   uint8_t instance_count)
{
    CanRxFrame_t frame;
    QueueSetMemberHandle_t ready;

    while ((ready = xQueueSelectFromSet(queues, 0U)) != NULL) {
        if (xQueueReceive(ready, &frame, 0U) != pdPASS) {
            continue;
        }

        for (uint8_t i = 0U; i < instance_count; ++i) {
            if (ready == instances[i].rx_queue) {
                if (instances[i].on_receive != NULL) {
                    instances[i].on_receive(instances[i].can, frame.id,
                                            frame.data, frame.length);
                } else {
                    CANReceive_Callback(instances[i].can, frame.id,
                                        frame.data, frame.length);
                }
                break;
            }
        }
    }
}

void ReceiveTask(void *argument)
{
    uint8_t instance_count = 0U;
    CanInstance_t *instances = CAN_GetInstances(&instance_count);
    QueueSetHandle_t queues;
    uint32_t notification_value;

    (void)argument;
    if (instances == NULL || instance_count == 0U) {
        vTaskDelete(NULL);
        return;
    }

    queues = xQueueCreateSet((UBaseType_t)instance_count *
                            (UBaseType_t)CAN_RX_QUEUE_LENGTH);
    if (queues == NULL) {
        vTaskDelete(NULL);
        return;
    }

    for (uint8_t i = 0U; i < instance_count; ++i) {
        if (instances[i].rx_queue == NULL ||
            xQueueAddToSet(instances[i].rx_queue, queues) != pdPASS) {
            vQueueDelete(queues);
            vTaskDelete(NULL);
            return;
        }
    }

    TaskReceive_InitProtocols();
    TaskReceive_StartUart();

    for (;;) {
        TaskReceive_ProcessCan(queues, instances, instance_count);

        for (uint8_t i = 0U; i < TASK_UART_INSTANCE_COUNT; ++i) {
            BSP_UART_t *uart = task_uart_instances[i];
            EventBits_t events = BSP_UART_GetEvents(uart, pdTRUE);

            if ((events & BSP_UART_EVENT_RX_READY) != 0U ||
                BSP_UART_RxAvailable(uart) != 0U) {
                TaskReceive_ProcessUart(uart);
            }
            if ((events & (BSP_UART_EVENT_RX_ERROR |
                          BSP_UART_EVENT_RX_OVERFLOW)) != 0U) {
                if (uart == &uart5_instance) {
                    RC_Remote_OnError(&rc_remote);
                } else if (uart == &uart1_instance) {
                    Referee_Reset(&referee_receiver);
                }
                Monitor_TaskError(MONITOR_TASK_RECEIVE);
            }
        }

        Monitor_TaskBeat(MONITOR_TASK_RECEIVE);
        notification_value = 0U;
        (void)xTaskNotifyWait(0U, (uint32_t)BSP_UART_EVENT_ALL,
                              &notification_value,
                              pdMS_TO_TICKS(TASK_RECEIVE_PERIOD_MS));
    }
}

#include "CAN_driver.h"

#include "task.h"

#include <stddef.h>
#include <string.h>

#define CAN_TX_REQUEST_ALL             0xFFFFFFFFU
#define CAN_TX_ERROR_RECOVER_THRESHOLD 128U
#define CAN_TX_FIFO_STUCK_POLLS        3U
#define CAN_RECOVER_COOLDOWN_MS         100U
#define CAN_RECOVER_COOLDOWN_TICKS     pdMS_TO_TICKS(CAN_RECOVER_COOLDOWN_MS)

/* ==================== 临时调试：追 0x146 丢在哪一步 ==================== */
/* 统计每个 ID 的入队、写入硬件 FIFO、接收次数，并记录失败事件。
 * 定位完成后把 CAN_DEBUG_TRACE 改成 0 就能整体关掉。 */
#define CAN_DEBUG_TRACE             1

#if CAN_DEBUG_TRACE
#define CAN_DEBUG_ID_SLOTS          32U   /* 扫描 0x141~0x14A 时 ID 槽会被占满，留够 */
#define CAN_DEBUG_EVENT_SLOTS       32U

typedef struct
{
    uint8_t  used;
    uint16_t id;
    uint8_t  rx_bus;            /* 最后收到该 ID 的 CAN 实例序号 */
    uint32_t enqueue_req;       /* 请求入队次数 */
    uint32_t enqueue_fail;      /* 队列满被丢 */
    uint32_t hw_ok;             /* 写进硬件 FIFO 成功 */
    uint32_t hw_fail;           /* HAL 拒绝 */
    uint32_t rx_count;          /* 中断里收到的帧数 */
    uint32_t last_tx_tick;
    uint32_t last_rx_tick;
} CanDebugIdStat_t;

typedef struct
{
    uint32_t tick;
    uint16_t id;
    uint8_t  kind;              /* 1=入队失败 2=硬件写入失败 3=发送前 FIFO 已满 */
    uint8_t  can_state;         /* 失败时的 hcan->State */
    uint16_t fifo_free;         /* 入队失败时为队列剩余，否则为 FIFO 空闲格数 */
} CanDebugEvent_t;

typedef struct
{
    CanDebugIdStat_t id[CAN_DEBUG_ID_SLOTS];
    CanDebugEvent_t  event[CAN_DEBUG_EVENT_SLOTS];
    uint32_t event_head;
    uint32_t event_count;
    uint32_t min_fifo_free;     /* 每次写硬件前读到的最小空闲格数 */
    uint32_t free_reads;
    uint32_t fifo_full_events;  /* 读到空闲为 0 的次数 */
    uint32_t tx_calls;
    uint32_t enqueue_fail_total;
    uint32_t hw_fail_total;
    uint32_t first_ok_tick;     /* 第一次成功写入硬件的时刻，0 表示还没有过 */
} CanDebug_t;

volatile CanDebug_t canDebug;

static volatile CanDebugIdStat_t *CanDebug_Slot(uint16_t id)
{
    volatile CanDebugIdStat_t *spare = NULL;

    for (uint8_t i = 0U; i < CAN_DEBUG_ID_SLOTS; ++i)
    {
        if (canDebug.id[i].used != 0U)
        {
            if (canDebug.id[i].id == id)
            {
                return &canDebug.id[i];
            }
        }
        else if (spare == NULL)
        {
            spare = &canDebug.id[i];
        }
    }

    if (spare != NULL)
    {
        spare->used = 1U;
        spare->id = id;
    }
    return spare;
}

/* 临时调试：清空逐 ID 统计/查询某 ID 的接收计数，供上层的对照实验用 */
void CanDebug_ResetIds(void)
{
    for (uint8_t i = 0U; i < CAN_DEBUG_ID_SLOTS; ++i)
    {
        canDebug.id[i].used = 0U;
        canDebug.id[i].id = 0U;
        canDebug.id[i].rx_count = 0U;
        canDebug.id[i].last_rx_tick = 0U;
    }
}

uint32_t CanDebug_RxCount(uint16_t id)
{
    for (uint8_t i = 0U; i < CAN_DEBUG_ID_SLOTS; ++i)
    {
        if (canDebug.id[i].used != 0U && canDebug.id[i].id == id)
        {
            return canDebug.id[i].rx_count;
        }
    }
    return 0U;
}

static void CanDebug_Event(uint16_t id, uint8_t kind, uint8_t can_state,
                           uint16_t detail)
{
    volatile CanDebugEvent_t *ev = &canDebug.event[canDebug.event_head];

    ev->tick = xTaskGetTickCount();
    ev->id = id;
    ev->kind = kind;
    ev->can_state = can_state;
    ev->fifo_free = detail;

    canDebug.event_head = (canDebug.event_head + 1U) % CAN_DEBUG_EVENT_SLOTS;
    canDebug.event_count++;
}
#endif

static CanInstance_t *can_registry;
static uint8_t can_registry_count;

/* 临时调试：中断栈(MSP)最低水位 */
volatile uint32_t g_isr_msp_min = 0xFFFFFFFFUL;

BaseType_t CAN_BindRx(CanInstance_t *instance, uint16_t id,
                      CanRxHandler_t handler, void *context)
{
    if (instance == NULL || handler == NULL ||
        instance->rx_binding_count >= CAN_RX_BINDING_MAX) {
        return pdFAIL;
    }

    CanRxBinding_t *binding = &instance->rx_binding[instance->rx_binding_count];
    binding->id = id;
    binding->handler = handler;
    binding->context = context;
    ++instance->rx_binding_count;
    return pdPASS;
}

void CAN_DispatchRx(const CanInstance_t *instance, uint16_t id,
                    const uint8_t *data, uint8_t length)
{
    for (uint8_t i = 0U; i < instance->rx_binding_count; ++i) {
        const CanRxBinding_t *binding = &instance->rx_binding[i];
        if (binding->id == id) {
            CAN_ISR_STACK_SAMPLE();
            binding->handler(binding->context, id, data, length);
        }
    }
}

void CAN_RegisterInstances(CanInstance_t *instances, uint8_t count)
{
    can_registry = instances;
    can_registry_count = (instances != NULL) ? count : 0U;
}

CanInstance_t *CAN_GetInstances(uint8_t *count)
{
    if (count != NULL) {
        *count = can_registry_count;
    }
    return can_registry;
}

uint8_t CAN_GetInstanceCount(void)
{
    return can_registry_count;
}

static CanInstance_t *CAN_InstanceFor(FDCAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || can_registry == NULL) {
        return NULL;
    }

    for (uint8_t i = 0U; i < can_registry_count; ++i) {
        if (can_registry[i].can == hcan) {
            return &can_registry[i];
        }
    }
    return NULL;
}

static QueueHandle_t CANTransmit_QueueFor(FDCAN_HandleTypeDef *hcan)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    return (instance != NULL) ? instance->tx_queue : NULL;
}

static HAL_StatusTypeDef CAN_ConfigFilter(const CanInstance_t *instance)
{
    if (instance == NULL || instance->can == NULL)
    {
        return HAL_ERROR;
    }

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    filter.FilterConfig = (instance->rx_fifo == FDCAN_RX_FIFO0) ?
                          FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;

    if (HAL_FDCAN_ConfigFilter(instance->can, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_FDCAN_ConfigGlobalFilter(instance->can,
                                        FDCAN_REJECT, FDCAN_REJECT,
                                        FDCAN_FILTER_REMOTE,
                                        FDCAN_FILTER_REMOTE);
}

void can_filter_init(void)
{
    if (can_registry == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < can_registry_count; ++i) {
        (void)CAN_ConfigFilter(&can_registry[i]);
    }
}

static HAL_StatusTypeDef CAN_ActivateNotifications(CanInstance_t *instance)
{
    if (instance == NULL || instance->can == NULL)
    {
        return HAL_ERROR;
    }

    uint32_t notification = (instance->rx_fifo == FDCAN_RX_FIFO0) ?
                            FDCAN_IT_RX_FIFO0_NEW_MESSAGE :
                            FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
    notification |= FDCAN_IT_BUS_OFF;
    return HAL_FDCAN_ActivateNotification(instance->can, notification, 0U);
}

void CANReceiveEnable(FDCAN_HandleTypeDef *hcan)
{
    (void)CAN_ActivateNotifications(CAN_InstanceFor(hcan));
}

void CANInitialize(FDCAN_HandleTypeDef *hcan)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    if (instance == NULL || instance->can == NULL) {
        return;
    }

    CAN_ConfigFilter(instance);
    if (HAL_FDCAN_Start(instance->can) != HAL_OK) {
        instance->monitor.err_reinit++;
        return;
    }
    CANReceiveEnable(instance->can);
}

void can_bsp_init(void)
{
    if (can_registry == NULL) {
        return;
    }

    can_filter_init();
    for (uint8_t i = 0U; i < can_registry_count; ++i) {
        CanInstance_t *instance = &can_registry[i];
        if (instance->can == NULL) {
            continue;
        }
        instance->monitor.tx_fifo_capacity = instance->can->Init.TxFifoQueueElmtsNbr;
        instance->monitor.tx_fifo_min_free = instance->monitor.tx_fifo_capacity;
        if (HAL_FDCAN_Start(instance->can) != HAL_OK) {
            instance->monitor.err_reinit++;
            continue;
        }
        CANReceiveEnable(instance->can);
    }
}

static void CAN_MonitorRefresh(CanInstance_t *instance)
{
    FDCAN_HandleTypeDef *hcan = instance->can;
    CANTxMonitor *monitor = &instance->monitor;
    FDCAN_ProtocolStatusTypeDef protocol_status;
    FDCAN_ErrorCountersTypeDef error_counters;

    monitor->tx_fifo_capacity = hcan->Init.TxFifoQueueElmtsNbr;
    monitor->tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(hcan);
    monitor->tx_fifo_used = monitor->tx_fifo_capacity > monitor->tx_fifo_free ?
                            monitor->tx_fifo_capacity - monitor->tx_fifo_free : 0U;
    if (monitor->tx_fifo_free < monitor->tx_fifo_min_free)
    {
        monitor->tx_fifo_min_free = monitor->tx_fifo_free;
    }
    if (monitor->tx_fifo_capacity != 0U && monitor->tx_fifo_free == 0U)
    {
        monitor->tx_fifo_full_count++;
    }
    else
    {
        monitor->tx_fifo_full_count = 0U;
    }

    monitor->rx_fifo0_capacity = hcan->Init.RxFifo0ElmtsNbr;
    if (monitor->rx_fifo0_capacity != 0U)
    {
        monitor->rx_fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(hcan, FDCAN_RX_FIFO0);
        monitor->rx_fifo0_free = monitor->rx_fifo0_capacity - monitor->rx_fifo0_fill;
        if (monitor->rx_fifo0_fill > monitor->rx_fifo0_peak_fill)
        {
            monitor->rx_fifo0_peak_fill = monitor->rx_fifo0_fill;
        }
    }
    monitor->rx_fifo1_capacity = hcan->Init.RxFifo1ElmtsNbr;
    if (monitor->rx_fifo1_capacity != 0U)
    {
        monitor->rx_fifo1_fill = HAL_FDCAN_GetRxFifoFillLevel(hcan, FDCAN_RX_FIFO1);
        monitor->rx_fifo1_free = monitor->rx_fifo1_capacity - monitor->rx_fifo1_fill;
        if (monitor->rx_fifo1_fill > monitor->rx_fifo1_peak_fill)
        {
            monitor->rx_fifo1_peak_fill = monitor->rx_fifo1_fill;
        }
    }

    if (HAL_FDCAN_GetProtocolStatus(hcan, &protocol_status) == HAL_OK)
    {
        monitor->last_error_code = protocol_status.LastErrorCode;
        monitor->data_last_error_code = protocol_status.DataLastErrorCode;
        monitor->activity = protocol_status.Activity;
        monitor->error_passive = protocol_status.ErrorPassive;
        monitor->warning = protocol_status.Warning;
        monitor->bus_off = protocol_status.BusOff;
    }
    if (HAL_FDCAN_GetErrorCounters(hcan, &error_counters) == HAL_OK)
    {
        monitor->tx_error_cnt = error_counters.TxErrorCnt;
        monitor->rx_error_cnt = error_counters.RxErrorCnt;
        monitor->rx_error_passive = error_counters.RxErrorPassive;
    }
    monitor->hal_error_code = hcan->ErrorCode;
}

static HAL_StatusTypeDef CAN_Recover(CanInstance_t *instance)
{
    FDCAN_HandleTypeDef *hcan = instance->can;

    if (hcan->State == HAL_FDCAN_STATE_BUSY)
    {
        (void)HAL_FDCAN_AbortTxRequest(hcan, CAN_TX_REQUEST_ALL);
        (void)HAL_FDCAN_Stop(hcan);
    }
    if (HAL_FDCAN_DeInit(hcan) != HAL_OK ||
        HAL_FDCAN_Init(hcan) != HAL_OK ||
        CAN_ConfigFilter(instance) != HAL_OK ||
        HAL_FDCAN_Start(hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return CAN_ActivateNotifications(instance);
}

void CAN_MonitorPoll(void)
{
    if (can_registry == NULL)
    {
        return;
    }

    for (uint8_t index = 0U; index < can_registry_count; ++index)
    {
        CanInstance_t *instance = &can_registry[index];
        CANTxMonitor *monitor = &instance->monitor;
        TickType_t now;
        uint8_t fifo_stuck;

        if (instance->can == NULL || instance->tx_mutex == NULL ||
            xSemaphoreTake(instance->tx_mutex, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        CAN_MonitorRefresh(instance);
        fifo_stuck = monitor->tx_fifo_full_count >= CAN_TX_FIFO_STUCK_POLLS &&
                     (monitor->error_passive != 0U || monitor->warning != 0U ||
                      monitor->tx_error_cnt >= CAN_TX_ERROR_RECOVER_THRESHOLD);
        if (monitor->bus_off != 0U || fifo_stuck != 0U ||
            instance->can->State != HAL_FDCAN_STATE_BUSY)
        {
            now = xTaskGetTickCount();
            if (monitor->last_recover_tick == 0U ||
                (TickType_t)(now - monitor->last_recover_tick) >= CAN_RECOVER_COOLDOWN_TICKS)
            {
                monitor->last_recover_tick = now;
                monitor->tx_fifo_full_count = 0U;
                monitor->last_recover_reason = monitor->bus_off != 0U ?
                                               CAN_RECOVER_REASON_BUS_OFF :
                                               (fifo_stuck != 0U ?
                                                CAN_RECOVER_REASON_TX_STUCK :
                                                CAN_RECOVER_REASON_NOT_RUNNING);
                monitor->last_recover_error_code = monitor->last_error_code;
                monitor->last_recover_hal_error_code = monitor->hal_error_code;
                monitor->last_recover_tx_error_cnt = monitor->tx_error_cnt;
                monitor->err_reinit++;
                if (CAN_Recover(instance) == HAL_OK)
                {
                    monitor->recover_success++;
                }
                else
                {
                    monitor->recover_fail++;
                }
                CAN_MonitorRefresh(instance);
            }
        }
        xSemaphoreGive(instance->tx_mutex);
    }
}

uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id,
                         const uint8_t *data, uint32_t len)
{
    CanInstance_t *instance = CAN_InstanceFor(hfdcan);
    HAL_StatusTypeDef send_status;
#if CAN_DEBUG_TRACE
    volatile CanDebugIdStat_t *dbg = CanDebug_Slot(id);
    uint16_t fifo_free = 0xFFFFU;
    uint8_t can_state = 0U;

    if (hfdcan != NULL)
    {
        can_state = (uint8_t)hfdcan->State;
        fifo_free = (uint16_t)HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
        canDebug.tx_calls++;
        if (canDebug.free_reads == 0U || fifo_free < canDebug.min_fifo_free)
        {
            canDebug.min_fifo_free = fifo_free;
        }
        canDebug.free_reads++;
        if (fifo_free == 0U)
        {
            canDebug.fifo_full_events++;
            CanDebug_Event(id, 3U, can_state, fifo_free);
        }
        if (dbg != NULL)
        {
            dbg->last_tx_tick = xTaskGetTickCount();
        }
    }
#endif
    if (instance != NULL && (id > CAN_STANDARD_ID_MAX || len > CAN_FRAME_DATA_MAX ||
                             (len != 0U && data == NULL))) {
        instance->monitor.tx_fail++;
        instance->monitor.last_fail_id = id;
        return 1U;
    }
    if (hfdcan == NULL || id > CAN_STANDARD_ID_MAX || len > CAN_FRAME_DATA_MAX ||
        (len != 0U && data == NULL)) {
        return 1U;
    }

    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    /* The HAL expects the DLC code (0..8), not a value shifted into a register. */
    header.DataLength = len;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    if (instance != NULL && instance->tx_mutex != NULL)
    {
        if (xSemaphoreTake(instance->tx_mutex, portMAX_DELAY) != pdTRUE)
        {
            instance->monitor.tx_fail++;
            instance->monitor.last_fail_id = id;
            return 1U;
        }
    }
    send_status = HAL_FDCAN_AddMessageToTxFifoQ(
        hfdcan, &header, (uint8_t *)(uintptr_t)data);
    if (instance != NULL && instance->tx_mutex != NULL)
    {
        xSemaphoreGive(instance->tx_mutex);
    }
    if (send_status != HAL_OK) {
        if (instance != NULL) {
            instance->monitor.tx_fail++;
            instance->monitor.last_fail_id = id;
        }
#if CAN_DEBUG_TRACE
        canDebug.hw_fail_total++;
        if (dbg != NULL) {
            dbg->hw_fail++;
        }
        CanDebug_Event(id, 2U, can_state, fifo_free);
#endif
        return 1U;
    }
    if (instance != NULL) {
        instance->monitor.tx_ok++;
    }
#if CAN_DEBUG_TRACE
    if (dbg != NULL) {
        dbg->hw_ok++;
        if (canDebug.first_ok_tick == 0U) {
            canDebug.first_ok_tick = xTaskGetTickCount();
        }
    }
#endif
    return 0U;
}

BaseType_t CANTransmit_QueueInit(void)
{
    if (can_registry == NULL || can_registry_count == 0U) {
        return pdFAIL;
    }

    for (uint8_t i = 0U; i < can_registry_count; ++i) {
        if (can_registry[i].tx_mutex == NULL)
        {
            can_registry[i].tx_mutex = xSemaphoreCreateMutex();
        }
        if (can_registry[i].tx_queue == NULL) {
            can_registry[i].tx_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH,
                                                     sizeof(CanTxFrame_t));
        }
        if (can_registry[i].tx_mutex == NULL ||
            can_registry[i].tx_queue == NULL)
        {
            return pdFAIL;
        }
    }
    return pdPASS;
}

/* 在中断里读空硬件 FIFO，每帧直接分发给绑定的回调，不经队列和任务
 * 即使实例没登记也要读空，否则 FIFO 满了会一直触发中断 */
static void CANReceive_FromIsr(FDCAN_HandleTypeDef *hcan, uint32_t fifo)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    FDCAN_RxHeaderTypeDef header;
    uint8_t rx_data[64];

    CAN_ISR_STACK_SAMPLE();

    if (hcan == NULL) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hcan, fifo) != 0U) {
        if (HAL_FDCAN_GetRxMessage(hcan, fifo, &header, rx_data) != HAL_OK) {
            break;
        }

        if (instance == NULL ||
            header.IdType != FDCAN_STANDARD_ID ||
            header.RxFrameType != FDCAN_DATA_FRAME ||
            header.DataLength > FDCAN_DLC_BYTES_8) {
            continue;
        }

        /* DLC 0 ~ 8 的编码值就等于字节数 */
        CAN_DispatchRx(instance, (uint16_t)header.Identifier, rx_data,
                       (uint8_t)header.DataLength);
#if CAN_DEBUG_TRACE
        {
            volatile CanDebugIdStat_t *dbg =
                CanDebug_Slot((uint16_t)header.Identifier);

            if (dbg != NULL)
            {
                dbg->rx_count++;
                dbg->rx_bus = (uint8_t)(instance - can_registry);
                dbg->last_rx_tick = xTaskGetTickCountFromISR();
            }
        }
#endif
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t flags)
{
    if ((flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        CANReceive_FromIsr(hcan, FDCAN_RX_FIFO0);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hcan, uint32_t flags)
{
    if ((flags & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U) {
        CANReceive_FromIsr(hcan, FDCAN_RX_FIFO1);
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hcan)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    if (instance != NULL) {
        instance->monitor.err_callback++;
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hcan, uint32_t flags)
{
    CanInstance_t *instance = CAN_InstanceFor(hcan);
    if (instance != NULL)
    {
        instance->monitor.err_status_callback++;
        if ((flags & FDCAN_IT_BUS_OFF) != 0U)
        {
            instance->monitor.bus_off = 1U;
        }
    }
}

BaseType_t CANTransmit_EnqueueLength(FDCAN_HandleTypeDef *hcan,
                                     uint16_t std_id, const uint8_t *data,
                                     uint8_t len)
{
    QueueHandle_t queue = CANTransmit_QueueFor(hcan);
    BaseType_t sent;
#if CAN_DEBUG_TRACE
    volatile CanDebugIdStat_t *dbg = CanDebug_Slot(std_id);
#endif

    if (queue == NULL || std_id > CAN_STANDARD_ID_MAX ||
        len > CAN_FRAME_DATA_MAX || (len != 0U && data == NULL)) {
        return pdFAIL;
    }

    CanTxFrame_t frame = {0};
    frame.can = hcan;
    frame.id = std_id;
    frame.length = len;
    if (len != 0U) {
        memcpy(frame.data, data, len);
    }
    sent = xQueueSend(queue, &frame, 0U);
#if CAN_DEBUG_TRACE
    if (dbg != NULL)
    {
        dbg->enqueue_req++;
        if (sent != pdPASS)
        {
            /* 队列满时这里原本是静默丢弃，现在记账 */
            dbg->enqueue_fail++;
            canDebug.enqueue_fail_total++;
            CanDebug_Event(std_id, 1U, 0U,
                           (uint16_t)uxQueueMessagesWaiting(queue));
        }
    }
#endif
    return sent;
}

BaseType_t CANTransmit_Enqueue(FDCAN_HandleTypeDef *hcan, uint16_t std_id,
                               const uint8_t data[CAN_FRAME_DATA_MAX])
{
    return CANTransmit_EnqueueLength(hcan, std_id, data,
                                     CAN_FRAME_DATA_MAX);
}

void CANTransmit_U8(FDCAN_HandleTypeDef *hcan, uint32_t std_id,
                    uint8_t data[])
{
    (void)fdcanx_send_data(hcan, (uint16_t)std_id, data,
                           CAN_FRAME_DATA_MAX);
}

void CANTransmit_I16(FDCAN_HandleTypeDef *hcan, uint32_t std_id,
                     int16_t output1, int16_t output2, int16_t output3,
                     int16_t output4)
{
    int16_t values[4] = {output1, output2, output3, output4};
    uint8_t data[CAN_FRAME_DATA_MAX];
    for (uint8_t i = 0U; i < 4U; ++i) {
        data[2U * i] = (uint8_t)((uint16_t)values[i] >> 8U);
        data[2U * i + 1U] = (uint8_t)values[i];
    }
    (void)fdcanx_send_data(hcan, (uint16_t)std_id, data,
                           CAN_FRAME_DATA_MAX);
}

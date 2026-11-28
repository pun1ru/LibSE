#ifndef MONITOR_H
#define MONITOR_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

/* Task heartbeat monitor.
 *
 * Tasks register through MonitorTaskConfig_t. A task only reports forward
 * progress with its MonitorHeartbeat_t token; timeout policy and event
 * handling stay in the monitor/application layers. Task notifications carry
 * one bit per registered task, so each task is counted at most once per
 * sampling period.
 */
#define MONITOR_TASK_MAX    16U

typedef struct Monitor Monitor_t;

typedef struct {
    Monitor_t *monitor;
    uint8_t task_id;
} MonitorHeartbeat_t;

typedef enum {
    MONITOR_EVENT_LOST = 1U,
    MONITOR_EVENT_RECOVERED = 2U
} MonitorEvent_t;

typedef struct MonitorTaskConfig MonitorTaskConfig_t;
typedef struct MonitorTaskStatus MonitorTaskStatus_t;

typedef void (*MonitorEventHandler_t)(
    MonitorEvent_t event,
    const MonitorTaskConfig_t *config,
    const MonitorTaskStatus_t *status,
    void *context);

struct MonitorTaskStatus {
    uint16_t beat_count;            /* Heartbeats seen in the current window */
    uint32_t offline_count;         /* Consecutive offline evaluations */
    volatile uint32_t last_heartbeat_tick;
    uint8_t online;
    /* Execution time statistics in CPU cycles (DWT CYCCNT) */
    uint32_t cycle_start;           /* CYCCNT at Monitor_CycleBegin */
    uint32_t cycle_last;            /* Last measured duration */
    uint32_t cycle_min;
    uint32_t cycle_max;
    uint32_t cycle_count;           /* Number of measurements since reset */
    uint64_t cycle_sum;             /* Sum for average: sum / count */
};

struct MonitorTaskConfig {
    const char *name;
    TaskHandle_t *task_handle;      /* Address is valid before task creation */
    uint16_t period_ms;
    uint16_t timeout_ms;
    MonitorEventHandler_t on_event;
    void *context;
    MonitorHeartbeat_t *heartbeat;  /* Filled by Monitor_Init */
};

typedef struct {
    TaskHandle_t *monitor_task;
} MonitorComm_t;

typedef struct {
    uint16_t sample_period_ms;
    uint8_t task_count;
    const MonitorTaskConfig_t *tasks;
} MonitorParam_t;

typedef struct {
    MonitorComm_t comm;
    MonitorParam_t param;
} MonitorConfig_t;

struct Monitor {
    MonitorConfig_t config;
    MonitorTaskStatus_t task[MONITOR_TASK_MAX];
    uint16_t sample_count;
};

BaseType_t Monitor_Init(Monitor_t *monitor, const MonitorConfig_t *config);

/* Call once after one processing iteration has completed successfully. */
void Monitor_Heartbeat(MonitorHeartbeat_t heartbeat);

/* Measure one iteration: call Begin at the start and End at the end.
 * Only use on periodic tasks; blocking waits are counted as run time. */
void Monitor_CycleBegin(MonitorHeartbeat_t heartbeat);
void Monitor_CycleEnd(MonitorHeartbeat_t heartbeat);
void Monitor_CycleReset(MonitorHeartbeat_t heartbeat);

/* The monitor task calls this once every sample period. */
void Monitor_Collect(Monitor_t *monitor);

/* Evaluate timeouts and dispatch LOST/RECOVERED callbacks.
 * Returns the number of tasks that transitioned to LOST. */
uint8_t Monitor_Evaluate(Monitor_t *monitor);

#endif

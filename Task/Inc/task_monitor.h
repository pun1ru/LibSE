#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include "monitor.h"

void MonitorTask(void *argument);
void MonitorTask_GetSnapshot(MonitorStatus_t *status);

#endif

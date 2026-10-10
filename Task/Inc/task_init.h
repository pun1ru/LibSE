#ifndef TASK_INIT_H
#define TASK_INIT_H

#include "FreeRTOS.h"

void CANInit(void);
void CommunicateInit(void);
void InitTask(void const *argument);

#endif


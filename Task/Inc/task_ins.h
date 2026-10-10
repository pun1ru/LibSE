#ifndef INS_TASK_H
#define INS_TASK_H

#include "ins.h"

extern INS_t board_ins;

typedef struct {
    uint32_t cycle_count;
    uint32_t update_count;
    uint32_t init_attempt_count;
    uint32_t init_error_count;
    float last_dt_s;
    uint8_t ready;
} INS_TaskStatus_t;

void INS_Task(void *argument);
void INS_Task_GetStatus(INS_TaskStatus_t *status);

#endif

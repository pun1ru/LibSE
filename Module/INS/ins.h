#ifndef INS_H
#define INS_H

#include "Madgwick.h"
#include <stdint.h>

typedef struct {
    MadgwickAHRS_t attitude;
    float gyro[3];
    float accel[3];
    float temperature;
    uint8_t ready;
    uint8_t init_status;
    uint32_t update_count;
    float last_dt_s;
} INS_t;

void INS_Init(INS_t *ins, float beta);
void INS_Update(INS_t *ins, float dt);

#endif

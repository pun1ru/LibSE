#include "ins.h"
#include "bmi088_driver.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>
//温控还没上
void INS_Init(INS_t *ins, float beta)
{
    if (ins == NULL) {
        return;
    }

    memset(ins, 0, sizeof(*ins));
    MadgwickAHRS_Init(&ins->attitude, beta);
    ins->init_status = BMI088_init();
    ins->ready = (ins->init_status == BMI088_NO_ERROR) ? 1U : 0U;
}

void INS_Update(INS_t *ins, float dt)
{
    if (ins == NULL || ins->ready == 0U || dt <= 0.0f) {
        return;
    }

    int16_t raw_gyro[3] = {0};
    int16_t raw_accel[3] = {0};
    int16_t raw_temp = 0;
    BMI088_read(raw_gyro, raw_accel, &raw_temp);
    for (uint8_t i = 0U; i < 3U; ++i) {
        ins->gyro[i] = (float)raw_gyro[i] * BMI088_GYRO_SEN;
        ins->accel[i] = (float)raw_accel[i] * BMI088_ACCEL_SEN;
    }
    ins->temperature = (float)raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    MadgwickAHRS_Update(&ins->attitude, ins->gyro[0], ins->gyro[1],
                        ins->gyro[2], ins->accel[0], ins->accel[1],
                        ins->accel[2], dt);
    taskENTER_CRITICAL();
    ins->euler.pitch_deg = ins->attitude.Pitch;
    ins->euler.roll_deg = ins->attitude.Roll;
    ins->euler.yaw_deg = ins->attitude.Yaw;
    for (uint8_t i = 0U; i < 4U; ++i) {
        ins->quat[i] = ins->attitude.q[i];
    }
    for (uint8_t i = 0U; i < 3U; ++i) {
        ins->gyro_snapshot[i] = ins->gyro[i];
    }
    taskEXIT_CRITICAL();
    ins->last_dt_s = dt;
    ++ins->update_count;
}

uint8_t INS_GetEuler(const INS_t *ins, INS_Euler_t *euler)
{
    if (ins == NULL || euler == NULL || ins->ready == 0U) {
        return 0U;
    }

    taskENTER_CRITICAL();
    *euler = ins->euler;
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t INS_GetQuat(const INS_t *ins, float q[4])
{
    if (ins == NULL || q == NULL || ins->ready == 0U) {
        return 0U;
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < 4U; ++i) {
        q[i] = ins->quat[i];
    }
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t INS_GetGyro(const INS_t *ins, float gyro_rads[3])
{
    if (ins == NULL || gyro_rads == NULL || ins->ready == 0U) {
        return 0U;
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < 3U; ++i) {
        gyro_rads[i] = ins->gyro_snapshot[i];
    }
    taskEXIT_CRITICAL();
    return 1U;
}

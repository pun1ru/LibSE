#include "ins.h"
#include "bmi088_driver.h"

#include <string.h>

void INS_Init(INS_t *ins, float beta)
{
    memset(ins, 0, sizeof(*ins));
    MadgwickAHRS_Init(&ins->attitude, beta);
    ins->ready = BMI088_init() == BMI088_NO_ERROR;
}

void INS_Update(INS_t *ins, float dt)
{
    if (ins->ready == 0U || dt <= 0.0f) {
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
}

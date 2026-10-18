#include "ins.h"
#include "bmi088_driver.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>
//温控还没上
void INS_Init(INS_t *ins, float beta, const float mount[3][3])
{
    if (ins == NULL || mount == NULL) {
        return;
    }

    memset(ins, 0, sizeof(*ins));
    memcpy(ins->mount, mount, sizeof(ins->mount));
    MadgwickAHRS_Init(&ins->attitude, beta);
    /* 机体系是 X 前 Y 右 Z 下，世界系 Z 向上，水平静止时机体到世界是绕 X 轴 180°，即 [0, 1, 0, 0]。
     * 默认的单位四元数与它正好差 180°，落在梯度下降的鞍点上，加速度修正几乎为 0，姿态收敛不了 */
    ins->attitude.q[0] = 0.0f;
    ins->attitude.q[1] = 1.0f;
    ins->attitude.q[2] = 0.0f;
    ins->attitude.q[3] = 0.0f;
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
    float gyro_sensor[3];
    float accel_sensor[3];
    for (uint8_t i = 0U; i < 3U; ++i) {
        gyro_sensor[i] = (float)raw_gyro[i] * BMI088_GYRO_SEN;
        accel_sensor[i] = (float)raw_accel[i] * BMI088_ACCEL_SEN;
    }
    /* 陀螺与加速度计必须用同一个安装矩阵，转到机体系后再进滤波器 */
    for (uint8_t i = 0U; i < 3U; ++i) {
        ins->gyro[i] = ins->mount[i][0] * gyro_sensor[0] +
                       ins->mount[i][1] * gyro_sensor[1] +
                       ins->mount[i][2] * gyro_sensor[2];
        ins->accel[i] = ins->mount[i][0] * accel_sensor[0] +
                        ins->mount[i][1] * accel_sensor[1] +
                        ins->mount[i][2] * accel_sensor[2];
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

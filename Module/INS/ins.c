#include "ins.h"
#include "bmi088_driver.h"

#include <string.h>

/* 机体系是 X 前 Y 右 Z 下，世界系 Z 向上，水平静止时机体到世界是绕 X 轴 180°，即 [0, 1, 0, 0]。
 * 默认的单位四元数与它正好差 180°，落在梯度下降的鞍点上，加速度修正几乎为 0，姿态收敛不了 */
static const float insInitQuat[4] = {0.0f, 1.0f, 0.0f, 0.0f};

//温控还没上
void INS_Init(INS_t *ins, const INSConfig_t *config)
{
    memset(ins, 0, sizeof(*ins));
    ins->comm = config->comm;
    memcpy(ins->mount, config->param.mount, sizeof(ins->mount));
    MadgwickAHRS_Init(&ins->attitude, config->param.beta);
    memcpy(ins->attitude.q, insInitQuat, sizeof(insInitQuat));
}

uint8_t INS_Connect(INS_t *ins)
{
    ins->init_status = BMI088_init();
    ins->ready = (ins->init_status == BMI088_NO_ERROR) ? 1U : 0U;
    return ins->ready;
}

/*--------------------------------------观测--------------------------------------*/
static void INS_Observe(INS_t *ins, float dt)
{
    int16_t raw_gyro[3] = {0};
    int16_t raw_accel[3] = {0};
    int16_t raw_temp = 0;
    float gyro_sensor[3];
    float accel_sensor[3];

    BMI088_read(raw_gyro, raw_accel, &raw_temp);
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
    ins->last_dt_s = dt;
    ++ins->update_count;
}

/*--------------------------------------通信--------------------------------------*/
static void INS_Communicate(const INS_t *ins)
{
    INSMsg_t msg;

    memcpy(msg.quat, ins->attitude.q, sizeof(msg.quat));
    memcpy(msg.gyro, ins->gyro, sizeof(msg.gyro));
    msg.euler.pitch_deg = ins->attitude.Pitch;
    msg.euler.roll_deg = ins->attitude.Roll;
    msg.euler.yaw_deg = ins->attitude.Yaw;
    msg.tick = xTaskGetTickCount();
    (void)xQueueOverwrite(ins->comm.msg_queue, &msg);
}

void INS_Run(INS_t *ins, float dt)
{
    if (ins->ready == 0U || dt <= 0.0f) {
        return;
    }

    INS_Observe(ins, dt);
    INS_Communicate(ins);
}

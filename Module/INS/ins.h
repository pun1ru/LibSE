#ifndef INS_H
#define INS_H

#include "Madgwick.h"
#include <stdint.h>

/* 欧拉角只读快照，单位 deg */
typedef struct {
    float pitch_deg;    ///< 俯仰角，抬头为正
    float roll_deg;     ///< 横滚角，右侧下沉为正
    float yaw_deg;      ///< 偏航角，顺时针为正
} INS_Euler_t;

typedef struct {
    MadgwickAHRS_t attitude;
    float gyro[3];
    float accel[3];
    float temperature;
    uint8_t ready;
    uint8_t init_status;
    uint32_t update_count;
    float last_dt_s;
    INS_Euler_t euler;      ///< 每次更新后整体写入的欧拉角快照
    float gyro_snapshot[3]; ///< 与 euler 同一时刻的机体角速度 rad/s
} INS_t;

void INS_Init(INS_t *ins, float beta);
void INS_Update(INS_t *ins, float dt);

/* 外部只读接口：拷贝一份当前欧拉角，INS 未就绪时返回 0 且不写 euler */
uint8_t INS_GetEuler(const INS_t *ins, INS_Euler_t *euler);
/* 外部只读接口：拷贝机体系角速度 [x, y, z]，单位 rad/s */
uint8_t INS_GetGyro(const INS_t *ins, float gyro_rads[3]);

#endif

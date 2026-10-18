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
    float mount[3][3];      ///< 传感器系 -> 机体系的安装旋转矩阵，v_body = mount * v_sensor
    float gyro[3];          ///< 机体系角速度 rad/s，已经过 mount
    float accel[3];         ///< 机体系加速度 m/s^2，已经过 mount
    float temperature;
    uint8_t ready;
    uint8_t init_status;
    uint32_t update_count;
    float last_dt_s;
    INS_Euler_t euler;      ///< 每次更新后整体写入的欧拉角快照
    float quat[4];          ///< 与 euler 同一时刻的四元数快照 [w, x, y, z]
    float gyro_snapshot[3]; ///< 与 euler 同一时刻的机体角速度 rad/s
} INS_t;

/* mount 必须是纯旋转矩阵（行列式 +1），第 i 行是机体第 i 轴在传感器系里的单位向量 */
void INS_Init(INS_t *ins, float beta, const float mount[3][3]);
void INS_Update(INS_t *ins, float dt);

/* 外部只读接口：拷贝一份当前欧拉角，INS 未就绪时返回 0 且不写 euler */
uint8_t INS_GetEuler(const INS_t *ins, INS_Euler_t *euler);
/* 外部只读接口：拷贝当前姿态四元数 [w, x, y, z]（机体到世界），INS 未就绪时返回 0 且不写 q */
uint8_t INS_GetQuat(const INS_t *ins, float q[4]);
/* 外部只读接口：拷贝机体系角速度 [x, y, z]，单位 rad/s */
uint8_t INS_GetGyro(const INS_t *ins, float gyro_rads[3]);

#endif

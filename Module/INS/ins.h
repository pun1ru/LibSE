#ifndef INS_H
#define INS_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "Madgwick.h"
#include "queue.h"

/* INS 只有观测和通信两层：INS_Run 读 BMI088、过滤波器，再把姿态快照发布到队列
 * 没有目标输入和控制层；BMI088 驱动自带固定的 SPI，所以外设层、电机层、状态层都为空 */

/* 欧拉角，单位 deg */
typedef struct {
    float pitch_deg;    ///< 俯仰角，抬头为正
    float roll_deg;     ///< 横滚角，右侧下沉为正
    float yaw_deg;      ///< 偏航角，顺时针为正
} INS_Euler_t;

/* 发布给其他模块的姿态快照，四个量取自同一次更新，读者整体取走即是同一时刻的数据 */
typedef struct {
    float quat[4];          ///< 机体到世界的四元数 [w, x, y, z]
    float gyro[3];          ///< 机体系角速度 rad/s，已经过 mount
    INS_Euler_t euler;
    TickType_t tick;        ///< 发布时刻
} INSMsg_t;

/*--------------------------------------通信层--------------------------------------*/
typedef struct {
    QueueHandle_t msg_queue;    ///< 长度 1 的覆盖式队列，元素为 INSMsg_t，由 System 层创建
} INSComm_t;

/*--------------------------------------参数层--------------------------------------*/
/* mount 必须是纯旋转矩阵（行列式 +1），第 i 行是机体第 i 轴在传感器系里的单位向量，v_body = mount * v_sensor */
typedef struct {
    float beta;
    float mount[3][3];
} INSParam_t;

typedef struct {
    INSComm_t comm;
    INSParam_t param;
} INSConfig_t;

/*--------------------------------------对象--------------------------------------*/
typedef struct {
    INSComm_t comm;
    MadgwickAHRS_t attitude;
    float mount[3][3];
    float gyro[3];          ///< 机体系角速度 rad/s，已经过 mount
    float accel[3];         ///< 机体系加速度 m/s^2，已经过 mount
    float temperature;
    uint8_t ready;
    uint8_t init_status;
    uint32_t update_count;
    float last_dt_s;
} INS_t;

/* 只构造对象，不碰硬件 */
void INS_Init(INS_t *ins, const INSConfig_t *config);
/* 初始化 BMI088，内部有忙等延时，不能放在关中断的地方；成功返回 1，失败可以重复调用 */
uint8_t INS_Connect(INS_t *ins);
/* 一个周期：观测（读传感器、滤波） -> 通信（发布姿态快照）；未连接时什么都不做 */
void INS_Run(INS_t *ins, float dt);

#endif

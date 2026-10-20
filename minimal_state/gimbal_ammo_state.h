#ifndef GIMBAL_AMMO_STATE_H
#define GIMBAL_AMMO_STATE_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "gimbal.h"
#include "queue.h"
#include "remote_control.h"

/* 最小状态机：只按遥控器拨杆决定云台状态，摇杆映射成世界系角速度，不处理键鼠
 * 右拨杆：上 -> 保护，中 -> 正常，下 -> 吊射；遥控器离线或拨杆值无效 -> 保护
 * 左摇杆：水平 ch2 -> yaw，竖直 ch3 -> pitch，方向沿用 INS 约定（右推 yaw 正，上推抬头）
 * 一个周期：GimbalAmmoState_Run，内部依次 目标输入（读遥控器）-> 观测（拨杆定状态）-> 控制（摇杆算角速度）-> 通信（发布命令） */

/*--------------------------------------外设层--------------------------------------*/
typedef struct {
    const RC_Remote_t *remote;  ///< 遥控器解析器，由 UART 接收回调喂数据
} GimbalAmmoStatePeriph_t;

/*--------------------------------------通信层--------------------------------------*/
typedef struct {
    QueueHandle_t cmd_queue;    ///< 元素 GimbalCmdMail_t，长度 1 覆盖式，由 System 层创建
} GimbalAmmoStateComm_t;

/*--------------------------------------状态层--------------------------------------*/
typedef struct {
    GimbalState_t init_state;
} GimbalAmmoStateInit_t;

/*--------------------------------------参数层--------------------------------------*/
typedef struct {
    float yaw_rate_dps;     /* 摇杆打满时的 yaw 角速度 */
    float pitch_rate_dps;   /* 摇杆打满时的 pitch 角速度 */
} GimbalStickGain_t;

typedef struct {
    GimbalStickGain_t normal;
    GimbalStickGain_t sniper;
    uint32_t remote_timeout_ms; /* 超过这个时间没收到遥控器数据按离线处理 */
} GimbalAmmoStateParam_t;

typedef struct {
    GimbalAmmoStatePeriph_t periph;
    GimbalAmmoStateComm_t comm;
    GimbalAmmoStateInit_t state;
    GimbalAmmoStateParam_t param;
} GimbalAmmoStateConfig_t;

typedef struct {
    GimbalAmmoStatePeriph_t periph;
    GimbalAmmoStateComm_t comm;
    GimbalAmmoStateParam_t param;
    GimbalState_t gimbal_state;
} GimbalAmmoState_t;

void GimbalAmmoState_Init(GimbalAmmoState_t *state, const GimbalAmmoStateConfig_t *config);
/* 运行函数，任务里每个周期调一次 */
void GimbalAmmoState_Run(GimbalAmmoState_t *state);

#endif

#ifndef GIMBAL_AMMO_STATE_H
#define GIMBAL_AMMO_STATE_H

#include <stdint.h>

#include "dt7_remote_driver.h"
#include "gimbal.h"

/* 最小状态机：只按遥控器拨杆决定云台状态，摇杆映射成世界系角速度，不处理键鼠
 * 右拨杆：上 -> 保护，中 -> 正常，下 -> 吊射；遥控器离线或拨杆值无效 -> 保护
 * 左摇杆：水平 ch2 -> yaw，竖直 ch3 -> pitch，方向沿用 INS 约定（右推 yaw 正，上推抬头） */

typedef struct {
    float yaw_rate_dps;     /* 摇杆打满时的 yaw 角速度 */
    float pitch_rate_dps;   /* 摇杆打满时的 pitch 角速度 */
} GimbalStickGain_t;

typedef struct {
    GimbalStickGain_t normal;
    GimbalStickGain_t sniper;
} GimbalAmmoStateConfig_t;

/* 状态机输出给云台的命令 */
typedef struct {
    GimbalState_t gimbal_state;
    float yaw_omega_dps;
    float pitch_omega_dps;
} GimbalAmmoCmd_t;

typedef struct {
    GimbalAmmoStateConfig_t config;
    GimbalState_t gimbal_state;
} GimbalAmmoState_t;

void GimbalAmmoState_Init(GimbalAmmoState_t *state, const GimbalAmmoStateConfig_t *config);
/* command 为 NULL 表示遥控器离线，直接进保护 */
void GimbalAmmoState_Update(GimbalAmmoState_t *state, const DT7CmdData *command,
                            GimbalAmmoCmd_t *output);
/* 在云台所在任务里调用：以云台当前目标为基准积分一个周期的角速度，再通过 Gimbal_Input 下发 */
void GimbalAmmoState_Apply(Gimbal_t *gimbal, const GimbalAmmoCmd_t *cmd, float period_s);

#endif

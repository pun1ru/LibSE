#include "gimbal_ammo_state.h"

#include <stddef.h>

#include "task.h"

/*--------------------------------------目标输入--------------------------------------*/
/* 还没收到有效帧或已超时返回 NULL，状态机据此进保护 */
static const DT7CmdData *GimbalAmmoState_ReadRemote(const GimbalAmmoState_t *state,
                                                    DT7CmdData *command)
{
    DT7RecData raw;
    uint32_t now_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (RC_Remote_Read(state->periph.remote, &raw, command) == 0U ||
        RC_Remote_IsOnline(state->periph.remote, now_ms, state->param.remote_timeout_ms) == 0U)
    {
        return NULL;
    }
    return command;
}

/*--------------------------------------观测--------------------------------------*/
static GimbalState_t GimbalAmmoState_FromSwitch(uint8_t switch_right)
{
    switch (switch_right)
    {
    case DT7_RC_SW_MID:
        return GIMBAL_NORMAL;
    case DT7_RC_SW_DOWN:
        return GIMBAL_SNIPER;
    default:
        return GIMBAL_PROTECTED;
    }
}

/*--------------------------------------控制--------------------------------------*/
/* command 为 NULL 表示遥控器离线，直接进保护 */
static GimbalCmd_t GimbalAmmoState_Update(GimbalAmmoState_t *state, const DT7CmdData *command)
{
    GimbalCmd_t cmd = {.state = GIMBAL_PROTECTED};

    state->gimbal_state = (command == NULL) ? GIMBAL_PROTECTED :
                          GimbalAmmoState_FromSwitch(command->switch_right);
    cmd.state = state->gimbal_state;
    if (state->gimbal_state == GIMBAL_PROTECTED)
    {
        return cmd;
    }

    const GimbalStickGain_t *gain = (state->gimbal_state == GIMBAL_SNIPER) ?
                                    &state->param.sniper : &state->param.normal;
    float stick_yaw = (float)command->ch2 / DT7_RC_CH_MAX_RELATIVE;
    float stick_pitch = (float)command->ch3 / DT7_RC_CH_MAX_RELATIVE;
    cmd.yaw_omega_dps = stick_yaw * gain->yaw_rate_dps;
    cmd.pitch_omega_dps = stick_pitch * gain->pitch_rate_dps;
    return cmd;
}

/*--------------------------------------通信--------------------------------------*/
static void GimbalAmmoState_Communicate(const GimbalAmmoState_t *state, const GimbalCmd_t *cmd)
{
    GimbalCmdMail_t mail =
    {
        .cmd = *cmd,
        .publish_tick = xTaskGetTickCount(),
    };
    (void)xQueueOverwrite(state->comm.cmd_queue, &mail);
}

/*--------------------------------------接口--------------------------------------*/
void GimbalAmmoState_Init(GimbalAmmoState_t *state, const GimbalAmmoStateConfig_t *config)
{
    state->periph = config->periph;
    state->comm = config->comm;
    state->param = config->param;
    state->gimbal_state = config->state.init_state;
}

void GimbalAmmoState_Run(GimbalAmmoState_t *state)
{
    DT7CmdData command;
    const DT7CmdData *remote = GimbalAmmoState_ReadRemote(state, &command);
    GimbalCmd_t cmd = GimbalAmmoState_Update(state, remote);

    GimbalAmmoState_Communicate(state, &cmd);
}

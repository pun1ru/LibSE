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

/* 参照 hero_up 的 fric_mode：左拨杆拨到上的那一下切换开关
 * 云台保护时摩擦轮也保护并清开关；上一帧没见过（离线后首帧）不算上升沿，防止恢复瞬间误开 */
static ShooterCmd_t GimbalAmmoState_UpdateShooter(GimbalAmmoState_t *state, const DT7CmdData *command)
{
    ShooterCmd_t cmd = {.state = SHOOTER_PROTECTED};

    if (command == NULL)
    {
        state->fric_on = 0U;
        state->last_switch_left = 0U;
    }
    else
    {
        uint8_t switch_left = (uint8_t)command->switch_left;
        if (state->gimbal_state == GIMBAL_PROTECTED)
        {
            state->fric_on = 0U;
        }
        else if (state->last_switch_left != 0U && state->last_switch_left != DT7_RC_SW_UP &&
                 switch_left == DT7_RC_SW_UP)
        {
            state->fric_on ^= 1U;
        }
        state->last_switch_left = switch_left;
    }

    if (state->gimbal_state != GIMBAL_PROTECTED)
    {
        cmd.state = (state->fric_on != 0U) ? SHOOTER_ENABLED : SHOOTER_STANDBY;
    }
    state->shooter_state = cmd.state;
    return cmd;
}

/*--------------------------------------通信--------------------------------------*/
static void GimbalAmmoState_Communicate(const GimbalAmmoState_t *state, const GimbalCmd_t *cmd,
                                        const ShooterCmd_t *shooter_cmd)
{
    TickType_t now = xTaskGetTickCount();
    GimbalCmdMail_t mail =
    {
        .cmd = *cmd,
        .publish_tick = now,
    };
    ShooterCmdMail_t shooter_mail =
    {
        .cmd = *shooter_cmd,
        .publish_tick = now,
    };
    (void)xQueueOverwrite(state->comm.cmd_queue, &mail);
    (void)xQueueOverwrite(state->comm.shooter_queue, &shooter_mail);
}

/*--------------------------------------接口--------------------------------------*/
void GimbalAmmoState_Init(GimbalAmmoState_t *state, const GimbalAmmoStateConfig_t *config)
{
    state->periph = config->periph;
    state->comm = config->comm;
    state->param = config->param;
    state->gimbal_state = config->state.init_state;
    state->shooter_state = SHOOTER_PROTECTED;
    state->fric_on = 0U;
    state->last_switch_left = 0U;
}

void GimbalAmmoState_Run(GimbalAmmoState_t *state)
{
    DT7CmdData command;
    const DT7CmdData *remote = GimbalAmmoState_ReadRemote(state, &command);
    GimbalCmd_t cmd = GimbalAmmoState_Update(state, remote);
    ShooterCmd_t shooter_cmd = GimbalAmmoState_UpdateShooter(state, remote);

    GimbalAmmoState_Communicate(state, &cmd, &shooter_cmd);
}

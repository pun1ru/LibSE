#include "gimbal_ammo_state.h"

#include <stddef.h>

/*--------------------------------------状态切换--------------------------------------*/
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

/*--------------------------------------接口--------------------------------------*/
void GimbalAmmoState_Init(GimbalAmmoState_t *state, const GimbalAmmoStateConfig_t *config)
{
    state->config = *config;
    state->gimbal_state = GIMBAL_PROTECTED;
}

void GimbalAmmoState_Update(GimbalAmmoState_t *state, const DT7CmdData *command,
                            GimbalAmmoCmd_t *output)
{
    output->yaw_omega_dps = 0.0f;
    output->pitch_omega_dps = 0.0f;

    if (command == NULL)
    {
        state->gimbal_state = GIMBAL_PROTECTED;
        output->gimbal_state = state->gimbal_state;
        return;
    }

    state->gimbal_state = GimbalAmmoState_FromSwitch(command->switch_right);
    output->gimbal_state = state->gimbal_state;
    if (state->gimbal_state == GIMBAL_PROTECTED)
    {
        return;
    }

    const GimbalStickGain_t *gain = (state->gimbal_state == GIMBAL_SNIPER) ?
                                    &state->config.sniper : &state->config.normal;
    float stick_yaw = (float)command->ch2 / DT7_RC_CH_MAX_RELATIVE;
    float stick_pitch = (float)command->ch3 / DT7_RC_CH_MAX_RELATIVE;
    output->yaw_omega_dps = stick_yaw * gain->yaw_rate_dps;
    output->pitch_omega_dps = stick_pitch * gain->pitch_rate_dps;
}

void GimbalAmmoState_Apply(Gimbal_t *gimbal, const GimbalAmmoCmd_t *cmd, float period_s)
{
    /* 保护态 Gimbal_Input 会忽略角度，target 由控制层跟随观测，退出保护时从当前姿态起积分 */
    GimbalInput_t input =
    {
        .state = cmd->gimbal_state,
        .yaw_deg = gimbal->target.yaw_deg + cmd->yaw_omega_dps * period_s,
        .pitch_deg = gimbal->target.pitch_deg + cmd->pitch_omega_dps * period_s,
        .yaw_omega_dps = cmd->yaw_omega_dps,
        .pitch_omega_dps = cmd->pitch_omega_dps,
    };
    Gimbal_Input(gimbal, &input);
}

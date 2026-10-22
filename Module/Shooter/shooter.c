#include "shooter.h"

#include <string.h>

#include "task.h"

/*--------------------------------------控制器管理--------------------------------------*/
/* 清空所有摩擦轮的 PID 和输出 */
static void Shooter_ResetControl(Shooter_t *shooter)
{
    for (uint8_t i = 0U; i < shooter->config.motor.wheel_count; ++i)
    {
        PIDReset(&shooter->controller.speed_pid[i]);
        shooter->controller.output[i] = 0.0f;
    }
}

void Shooter_Init(Shooter_t *shooter, const ShooterConfig_t *config)
{
    memset(shooter, 0, sizeof(*shooter));
    shooter->config = *config;
    shooter->state = config->state.init_state;

    const ShooterPIDParam_t *pid = &config->param.speed;
    for (uint8_t i = 0U; i < config->motor.wheel_count; ++i)
    {
        Motor_t *motor = config->motor.wheel[i];
        /* 反馈帧绑到各自的 CAN 实例，回调只把数据解析进电机对象 */
        (void)Motor_BindCan(motor, config->periph.can[i]);
        PIDInitialize(&shooter->controller.speed_pid[i], pid->kp, pid->ki, pid->kd,
                      pid->d_alpha, pid->sum_error_max, pid->output_max);
        /* 还没见过反馈之前按失联处理 */
        shooter->observer.stale_count[i] = config->param.feedback_timeout_cycles;
    }
}

/*--------------------------------------输入层--------------------------------------*/
void Shooter_Input(Shooter_t *shooter, const ShooterCmd_t *cmd)
{
    if (cmd->state != shooter->state)
    {
        shooter->state = cmd->state;
        Shooter_ResetControl(shooter);
    }
}

/*--------------------------------------估计层--------------------------------------*/
/* 电机 inverse 只翻转指令，反馈在这里按同一方向折算成发射方向的转速
 * 反馈计数不再变化说明电机或 CAN 失联，积累到超时就让观测无效 */
void Shooter_Estimate(Shooter_t *shooter)
{
    ShooterObserver_t *obs = &shooter->observer;
    uint16_t timeout_cycles = shooter->config.param.feedback_timeout_cycles;
    uint8_t all_alive = 1U;

    for (uint8_t i = 0U; i < shooter->config.motor.wheel_count; ++i)
    {
        const Motor_t *motor = shooter->config.motor.wheel[i];
        obs->speed_rpm[i] = Motor_Direction(motor) * motor->feedback.velocity_rpm;

        if (motor->feedback.timestamp != obs->feedback_stamp[i])
        {
            obs->feedback_stamp[i] = motor->feedback.timestamp;
            obs->stale_count[i] = 0U;
        }
        else if (obs->stale_count[i] < timeout_cycles)
        {
            ++obs->stale_count[i];
        }
        if (obs->stale_count[i] >= timeout_cycles)
        {
            all_alive = 0U;
        }
    }
    obs->valid = all_alive;
}

/*--------------------------------------控制层--------------------------------------*/
/* 非保护状态下所有摩擦轮共用的目标转速 */
static float Shooter_TargetRpm(const Shooter_t *shooter)
{
    return (shooter->state == SHOOTER_ENABLED) ? shooter->config.param.fric_rpm :
                                                 shooter->config.param.standby_rpm;
}

void Shooter_Control(Shooter_t *shooter)
{
    if (shooter->state == SHOOTER_PROTECTED || shooter->observer.valid == 0U)
    {
        Shooter_ResetControl(shooter);
        return;
    }

    float target_rpm = Shooter_TargetRpm(shooter);
    for (uint8_t i = 0U; i < shooter->config.motor.wheel_count; ++i)
    {
        /* PIDUpdate 内部已按 output_max 限幅 */
        shooter->controller.output[i] = PIDUpdate(&shooter->controller.speed_pid[i],
                                                  target_rpm - shooter->observer.speed_rpm[i]);
    }
}

/*--------------------------------------目标输入--------------------------------------*/
/* 取最近一条命令；队列为空或超时未更新按保护处理 */
static void Shooter_TargetInput(Shooter_t *shooter)
{
    ShooterCmdMail_t mail;
    ShooterCmd_t cmd = {.state = SHOOTER_PROTECTED};

    if (xQueuePeek(shooter->config.comm.cmd_queue, &mail, 0U) == pdPASS &&
        (TickType_t)(xTaskGetTickCount() - mail.publish_tick) <=
            pdMS_TO_TICKS(shooter->config.param.cmd_timeout_ms))
    {
        cmd = mail.cmd;
    }
    Shooter_Input(shooter, &cmd);
}

/*--------------------------------------通信--------------------------------------*/
/* 先写完所有轮子再发送：同组 RM 电机共用一帧，逐个写+发会带出旧值
 * 所有摩擦轮约定在同一组，一帧带全部指令，所以只经第一个轮子发一次 */
static void Shooter_Communicate(Shooter_t *shooter)
{
    const ShooterMotor_t *motors = &shooter->config.motor;
    for (uint8_t i = 0U; i < motors->wheel_count; ++i)
    {
        motors->wheel[i]->motorcmd.current = shooter->controller.output[i];
    }
    Motor_t *first = motors->wheel[0];
    first->motorops.sendgroupcommand(first);
}

void Shooter_Run(Shooter_t *shooter)
{
    Shooter_TargetInput(shooter);
    Shooter_Estimate(shooter);
    Shooter_Control(shooter);
    Shooter_Communicate(shooter);
}

#include "Motor.h"

#include "task.h"

#include "algorism.h"

void Motor_UpdateFeedbackAngle(Motor_t *motor, float angle_rad)
{
    Feedback_t *feedback = &motor->feedback;
    uint32_t now_ms = HAL_GetTick();
    float abs_angle_rad = AngleLimit(angle_rad, -MOTOR_PI, MOTOR_PI);

    if (motor->feedback_angle_valid == 0U ||
        (uint32_t)(now_ms - motor->last_feedback_ms) > MOTOR_FEEDBACK_TIMEOUT_MS)
    {
        motor->multi_turn_count = 0;
    }
    else
    {
        float delta = abs_angle_rad - motor->last_abs_angle_rad;

        if (delta > MOTOR_PI)
        {
            motor->multi_turn_count--;
        }
        else if (delta < -MOTOR_PI)
        {
            motor->multi_turn_count++;
        }
    }

    motor->last_abs_angle_rad = abs_angle_rad;
    motor->last_feedback_ms = now_ms;
    motor->feedback_angle_valid = 1U;
    feedback->abs_angle_rad = abs_angle_rad;
    feedback->abs_angle_deg = abs_angle_rad * MOTOR_RAD_TO_DEG;
    feedback->multi_angle_rad = (float)motor->multi_turn_count * MOTOR_TWO_PI +
                                abs_angle_rad;
    feedback->multi_angle_deg = (float)motor->multi_turn_count * 360.0f +
                                feedback->abs_angle_deg;
}

void Motor_UpdateFeedbackStatus(Motor_t *motor)
{
    uint32_t now_ms = HAL_GetTick();

    taskENTER_CRITICAL();
    if (motor->feedback_angle_valid != 0U &&
        (uint32_t)(now_ms - motor->last_feedback_ms) > MOTOR_FEEDBACK_TIMEOUT_MS)
    {
        motor->feedback_angle_valid = 0U;
        motor->multi_turn_count = 0;
        motor->feedback.multi_angle_rad = motor->feedback.abs_angle_rad;
        motor->feedback.multi_angle_deg = motor->feedback.abs_angle_deg;
        motor->feedback.status = MOTOR_NO_FEEDBACK;
    }
    taskEXIT_CRITICAL();
}

/* 回调只做长度检查，交给电机自己的解析函数，不碰外部对象 */
static void Motor_OnCanRx(void *context, uint16_t id, const uint8_t *data, uint8_t length)
{
    Motor_t *motor = (Motor_t *)context;

    if (length == CAN_FRAME_DATA_MAX)
    {
        motor->motorops.getfeedback(motor, data, id);
    }
}

bool Motor_BindCan(Motor_t *motor, CanInstance_t *can)
{
    return CAN_BindRx(can, motor->feedback_id, Motor_OnCanRx, motor) == pdPASS;
}

void Motor_BaseInit(Motor_t *motor, FDCAN_HandleTypeDef *can,
                    uint16_t zero_point, bool inverse, float gear_ratio)
{
    motor->can = can;
    motor->zero_point = zero_point;
    motor->inverse = inverse;
    motor->gear_ratio = gear_ratio;
    motor->feedback.status = MOTOR_NO_FEEDBACK;
}

#include "Motor.h"

#include "algorism.h"

void Motor_SetAbsAngle(Feedback_t *feedback, float angle_rad)
{
    feedback->abs_angle_rad = AngleLimit(angle_rad, -MOTOR_PI, MOTOR_PI);
    feedback->abs_angle_deg = feedback->abs_angle_rad * MOTOR_RAD_TO_DEG;
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

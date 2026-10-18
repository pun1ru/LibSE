#include "Motor.h"

#include "algorism.h"

void Motor_SetAbsAngle(Feedback_t *feedback, float angle_rad)
{
    feedback->abs_angle_rad = AngleLimit(angle_rad, -MOTOR_PI, MOTOR_PI);
    feedback->abs_angle_deg = feedback->abs_angle_rad * MOTOR_RAD_TO_DEG;
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

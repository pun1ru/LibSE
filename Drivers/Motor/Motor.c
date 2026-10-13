#include "Motor.h"

float Motor_WrapAngleRad(float angle_rad)
{
    while (angle_rad > MOTOR_PI)
    {
        angle_rad -= MOTOR_TWO_PI;
    }
    while (angle_rad <= -MOTOR_PI)
    {
        angle_rad += MOTOR_TWO_PI;
    }
    return angle_rad;
}

void Motor_SetAbsAngle(Feedback_t *feedback, float angle_rad)
{
    feedback->abs_angle_rad = Motor_WrapAngleRad(angle_rad);
    feedback->abs_angle_deg = feedback->abs_angle_rad * MOTOR_RAD_TO_DEG;
}

#include "Motor.h"

#include "algorism.h"

void Motor_SetAbsAngle(Feedback_t *feedback, float angle_rad)
{
    feedback->abs_angle_rad = AngleLimit(angle_rad, -MOTOR_PI, MOTOR_PI);
    feedback->abs_angle_deg = feedback->abs_angle_rad * MOTOR_RAD_TO_DEG;
}

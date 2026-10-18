#include "math.h"

#include "pid.h"

void PIDInitialize(PIDStruct *const pid, float kp, float ki, float kd, float d_alpha, float sum_error_max, float output_max)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->d_alpha = d_alpha;

	pid->cur_error = 0;
	pid->last_error = 0;
	pid->last_last_error = 0;
	pid->deriv_filtered = 0;//滤波项输出
	pid->sum_error = 0;
	pid->output = 0;


	pid->sum_error_max = sum_error_max;//积分限幅
	pid->output_max = output_max;//输出限幅
}
//滤波更新
static float PIDFilteredDerivative(PIDStruct *const pid)
{
	pid->deriv_filtered = pid->deriv_filtered * pid->d_alpha +\
												(pid->cur_error - pid->last_error) * (1.0f - pid->d_alpha);
	return pid->deriv_filtered;
}

float PIDUpdate(PIDStruct *const pid, const float error)
{
	pid->last_error = pid->cur_error;
	pid->cur_error = error;
	pid->sum_error += pid->cur_error;
	pid->sum_error = limiter(pid->sum_error, pid->sum_error_max);
	pid->output = pid->kp * pid->cur_error +\
								pid->ki * pid->sum_error +\
								pid->kd * PIDFilteredDerivative(pid);
	return pid->output = limiter(pid->output, pid->output_max);
}


float PIDUpdatePrior(PIDStruct pid, const float error)
{
	return PIDUpdate(&pid, error);
}
//积分分离板pid,error_limit大误差不累计i项
float PIDUpdateIntergrateSeparation(PIDStruct* const pid, const float error, const float error_limit)
{
	pid->last_error = pid->cur_error;
	pid->cur_error = error;
	if(fabsf(error) < fabsf(error_limit))
	{
		pid->sum_error += pid->cur_error;
		pid->sum_error = limiter(pid->sum_error, pid->sum_error_max);
	}
	pid->output = pid->kp * pid->cur_error +\
								pid->ki * pid->sum_error +\
								pid->kd * PIDFilteredDerivative(pid);
	return pid->output = limiter(pid->output, pid->output_max);
}

//增量式pid
float PIDUpdateIncrement(PIDStruct* const pid, float current_error)
{
	pid->last_last_error = pid->last_error;
	pid->last_error = pid->cur_error;
	pid->cur_error = current_error;
	pid->output = pid->kp * (pid->cur_error - pid->last_error) \
				 + pid->ki * pid->cur_error \
				 + pid->kd * (pid->cur_error - 2 * pid->last_error + pid->last_last_error);
	pid->output = limiter(pid->output, pid->output_max);
	return pid->output;
}

void PIDReset(PIDStruct *const pid)
{
	pid->cur_error = 0;
	pid->last_error = 0;
	pid->last_last_error = 0;
	pid->sum_error = 0;
	pid->deriv_filtered = 0;
	pid->output = 0;
}

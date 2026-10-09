#include "math.h"

#include "pid.h"
/**
* \brief pid initialize
* \param[in] pid PID struct to be initialized
* \param[in] kp Proportional coefficient
* \param[in] ki Integral coefficient
* \param[in] kd Derivative coefficient
* \param[in] sum_error_max Maximum value of the integral term
* \param[in] output_max Maximum value of the output
*/
void PIDInitialize(PIDStruct *const pid, float kp, float ki, float kd, float sum_error_max, float output_max)
{
	//< pid parameters
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	//< clear buffers
	pid->cur_error = 0;
	pid->last_error = 0;
	pid->sum_error = 0;
	pid->deriv_filtered = 0;
	
	//< constrain parameters
	pid->sum_error_max = sum_error_max;
	pid->output_max = output_max;
}

/**
* \brief Compute pid output(normalize)
* \param[in] pid The  pid struct to be updated
* \param[in] error Calculated error of the control system (target - feedback)
* \return Control output
*/
float PIDUpdate(PIDStruct *const pid, const float error)
{
	pid->last_error = pid->cur_error;
	pid->cur_error = error;
	pid->sum_error += pid->cur_error;
	pid->sum_error = limiter(pid->sum_error, pid->sum_error_max);
	pid->output = pid->kp * pid->cur_error +\
								pid->ki * pid->sum_error +\
								pid->kd * (pid->cur_error - pid->last_error);
	return pid->output = limiter(pid->output, pid->output_max);
}

/**
* \brief Compute pid output with low-pass filtered D term
* \param[in] pid The pid struct to be updated
* \param[in] error Calculated error of the control system (target - feedback)
* \param[in] alpha Low-pass coefficient of D term (0~1, larger = heavier filtering)
* \return Control output
* \note  D term: deriv_f = deriv_f * alpha + (cur_error - last_error) * (1 - alpha)
*/
float PIDUpdateWithFilteredD(PIDStruct *const pid, const float error, const float alpha)
{
	pid->last_error = pid->cur_error;
	pid->cur_error = error;
	pid->sum_error += pid->cur_error;
	pid->sum_error = limiter(pid->sum_error, pid->sum_error_max);
	pid->deriv_filtered = pid->deriv_filtered * alpha + (pid->cur_error - pid->last_error) * (1.0f - alpha);
	pid->output = pid->kp * pid->cur_error + pid->ki * pid->sum_error + pid->kd * pid->deriv_filtered;
	return pid->output = limiter(pid->output, pid->output_max);
}

/**
* \brief Compute pid output but do not change the value of pid struct
* \param[in] pid The pid struct to be copied
* \param[in] error Calculated error of the control system (target - feedback)
* \return Control output
*/
float PIDUpdatePrior(PIDStruct pid, const float error)
{
	pid.last_error = pid.cur_error;
	pid.cur_error = error;
	pid.sum_error += pid.cur_error;
	pid.sum_error = limiter(pid.sum_error, pid.sum_error_max);
	pid.output = pid.kp * pid.cur_error +\
								pid.ki * pid.sum_error +\
								pid.kd * (pid.cur_error - pid.last_error);
	return pid.output = limiter(pid.output, pid.output_max);
}

/**
* \brief clear pid buffer
* \param[in] pid The  pid struct to be updated
* \return void
*/
void PIDRefreshBuffer(PIDStruct* const pid)
{
	pid->cur_error = 0;
	pid->last_error = 0;
	pid->sum_error = 0;
	pid->deriv_filtered = 0;
	pid->output = 0;
}

//TODO: optimize PID calculation
/**
* \brief integration refresh when abs_error greater than limit_value
* \param[in] pid The  pid struct to be updated
* \param[in] error Calculated error of the control system (target - feedback)
* \param[in] error_limit Intergration refresh when the |current_error| > |error_limit|
* \return void
*/
float PIDUpdateIntergrateRefresh(PIDStruct* const pid, const float error, const float error_limit)
{	
	pid->last_error = pid->cur_error;
	pid->cur_error = error;
	pid->sum_error += pid->cur_error;
	pid->sum_error = limiter(pid->sum_error, pid->sum_error_max);
	if(fabs(error) > fabs(error_limit)) pid->sum_error = 0;
	pid->output = pid->kp * pid->cur_error +\
								pid->ki * pid->sum_error +\
								pid->kd * (pid->cur_error - pid->last_error);
	return pid->output = limiter(pid->output, pid->output_max);	
}
/**
* \param[in] pid The  pid struct to be updated
* \param[in] error Calculated error of the control system (target - feedback)
* \param[in] error_limit use pd control when the |current_error| < |error_limit|, otherwise use pid control
* \return void
*/
float PIDUpdateIntergrateSeparation(PIDStruct* const pid, const float error, const float error_limit)
{
	pid->last_error = pid->cur_error;
	pid->cur_error = error;
	pid->sum_error += pid->cur_error;
	pid->sum_error = limiter(pid->sum_error, pid->sum_error_max);
	float beta = 0;
	if(fabs(error) < fabs(error_limit))	beta = 0;
	pid->output = pid->kp * pid->cur_error +\
								pid->ki * pid->sum_error * beta +\
								pid->kd * (pid->cur_error - pid->last_error);
	return pid->output = limiter(pid->output, pid->output_max);
	}

/**
 * \brief Incremental PID control function
 *
 * \param pid Pointer to PIDStruct containing PID parameters and variables
 * \param current_error Current error for the PID controller
 * \return Incremental output of the PID controller
 */
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

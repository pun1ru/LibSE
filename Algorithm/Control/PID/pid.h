#ifndef _PID_H_
#define _PID_H_

#include "stdint.h"

#define limiter(val,max) ((val)<(-max)?(-max):((val)>(max)?(max):(val)))

/// @brief pid计算结构体
typedef struct {
	/**
	* \brief pid parameters
	*/
	float kp, ki, kd;
	
	/**
	* \brief errors
	*/
	float cur_error, last_error, last_last_error, sum_error;
	
	/**
	* \brief pid output
	*/
	float output;
	/**
	* \brief filtered derivative term (D low-pass)
	*/
	float deriv_filtered;
	
	/**
	* \brief Constrain parameters
	*/
	float sum_error_max, output_max;
}PIDStruct;

void PIDRefreshBuffer(PIDStruct* const pid);
void PIDReset(PIDStruct *const pid);
void PIDInitialize(PIDStruct *const pid, float kp, float ki, float kd, float sum_error_max, float output_max);
float PIDUpdate(PIDStruct *const pid, const float error);
float PIDUpdatePrior(PIDStruct pid, const float error);
float PIDUpdateWithFilteredD(PIDStruct *const pid, const float error, const float alpha);

/*optimize pid compute*/
float PIDUpdateIntergrateRefresh(PIDStruct* const pid, const float error, const float error_limit);
#endif

#ifndef _ADRC_H_
#define _ADRC_H_

#include "stdint.h"

/**
 * @brief TD跟踪微分器实现(Tracking Differentiator)
 * @note 由于该过滤器存在严重的相角滞后，一般该微分器适用于处理输入信号而不适合处理反馈信号
 */
typedef struct
{
	float input;
	/*信号一阶及二阶跟踪量*/
	float x1;
	float x2;

	/*系统参数*/
	float r;//速度因子--值越大，逼近速度越快，信号更趋近于真实曲线，有棱有角，取决于受控对象能否承受
	float h;//积分步长，即运行周期
	uint8_t N;//积分步长扩大系数，用于增大h，减少震荡，一般N*h增大，能适当抑制噪声
	/*处理周期信号的上下限*/
	float min;
	float max;
}TD;
typedef struct
{
	float input;
	/*信号一阶及二阶跟踪量*/
	float x1;
	float x2;

	/*系统参数*/
	float r;//速度因子--值越大，逼近速度越快，信号更趋近于真实曲线，有棱有角，取决于受控对象能否承受
	float h;//积分步长，即运行周期
	float min;
	float max;
	uint32_t cnt;
	
	/*一些工程上的配置参数，只看LTD可以无视，对于本车结合使用请*/
	float kp1;
	float kp2;
	float kd1;
	float kd2;
	float ki1;
	float ki2;
	float error_sum;
	float error_sum_max;
	float lv_bo;
	float out_put_limit;
}LTD;

typedef struct
{
	float kp;
	float _pitchkd;
	float w_d_limit;
	float p_output_limit;
	
}LTDPID;
static float fhan(float x1, float x2, float r, float h);
void TDInitialize(TD* td, float r, float h0, float N, float min, float max);
void TDSetParam(TD* td, float r, float h0, float N, float min, float max);
void TD_Reset(TD* td, float x1);
void TDUpdate(TD* td, float target);
void LTDInitialize(LTD* ltd, float r, float h, float min, float max);
void LTDSetParam(LTD* ltd, float r, float h, float min, float max);
void LTD_Reset(LTD* ltd, float x1);
void LTDUpdate(LTD* ltd, float target);
void LTDUpdateNoLimit(LTD* ltd, float target);
void LTDPIDInitialize(LTDPID* ltdpid,float kp,float _pitchkd,float w_d_limit,float p_output_limit);
/**
 * @brief 拓张状态观测器，估计扰动
 */
typedef struct
{
	/*系统观测量*/
	float z1;
	float z2;
	float z3;
	
	/*系统参数--根据带宽法整定参数，取beta_01 = 1/h beta_02 = 1/(3*h*h) beta_03=1/(20*h*h*h)*/
	float h;//积分步长
	float b;//控制量系数，控制量z2计算式叠加了一个b*u的项，u即为控制量	
	float beta_01;//估计一阶量z1时非线性函数扩大系数
	float beta_02;//估计二阶量z2时非线性函数扩大系数
	float beta_03;//估计三阶量z3时非线性函数扩大系数

	/*处理周期信号的上下限及限幅*/
	float z3_limit;	
	float min;
	float max;
	float z1_min;
	float z1_max;
	float z2_min;
	float z2_max;
	

}ESO;

float Fal(float e, float alpha, float delta);
void TDInitialize(TD* td, float r, float h0, float N, float min, float max);
void TDUpdate(TD* td, float target);
void ESOInitialize(ESO* eso, float h, float b, float b_01, float b_02, float b_03, float z3_limit, float min, float max);
void ESOUpdate(ESO* eso, float feedback, float control_val);
void LESOUpdate(ESO* eso, float feedback, float control_val);
void ESO_Reset(ESO* eso, float z1);

/// @brief 误差输出组合器
typedef struct
{
	float e_0;//误差累计
	float e_1;
	float e_2;
	float output;//当前输出
	
	float k_0;//积分项扩大系数
	float k_1;//误差一阶项扩大系数
	float k_2;//误差二阶项扩大系数

	float e_0_max;//误差累计上限
	float output_limit;//输出上限
}ESF;

//线性组合等同于PID
void LESFInitialize(ESF* esf, float k_0, float k_1, float k_2, float e_0_max, float output_limit);
float LESFUpdate(ESF* esf, float e_1, float e_2);

typedef struct
{
	TD td;
	ESO eso;
	ESF esf;
	float u;//补偿后控制量
	float u_0;//补偿前控制量
	float limit_max;//处理周期信号上限
	float limit_min;//处理周期信号下限
}ADRC;
void LADRCInitialize(ADRC* adrc, float* td_init_val, float* lesf_init_val, float* eso_init_val, float min, float max);
void LADRCUpdate(ADRC* adrc, float target, float feedback);
void YawLADRCUpdate(ADRC* adrc, float target, float feedback);
void LADRCUpdateV2(ADRC* adrc, float target, float feedback, float actual_velocity, float z3_gain);
void LTDADRCUpdate(ADRC* adrc,LTD* ltd, float target, float feedback);

#endif

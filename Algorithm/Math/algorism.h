#ifndef __ALGORISM_H
#define __ALGORISM_H	

#include <stdint.h>

#define square(x) x*x
#define cube(x) x*x*x
#define fsgn(x) ( (fabs(x)<1e-6) ? 0:(0<x)-(x<0) )


/**
 * \brief 平滑滤波器结构体 
 */
typedef struct
{
    float alpha; 
    float last, current;
}SmoothFilter;
typedef struct
{
	float last[3];
}AverageFilter;

/**
 * \brief 标量卡尔曼滤波器（1状态 + 1控制 + 1测量）
 * \note  模型: x_k = x_{k-1} + u * dt + w,  z_k = x_k + v
 *         适用于编码器角度融合角速度的场景
 */
typedef struct
{
    float x;        /**< 状态估计值（角度 deg） */
    float P;        /**< 误差协方差 */
    float Q;        /**< 过程噪声协方差 */
    float R;        /**< 测量噪声协方差 */
    float dt;       /**< 采样周期（秒） */
    uint8_t init;   /**< 首次初始化标志 */
}ScalarKalmanFilter;

typedef struct
{
	float x[30];
	float y[30];
	uint16_t num;
	float a;
	float b;
	float valid_num;
	uint8_t count;
}leastSquareLinear;

void SmoothFilterInitialize(SmoothFilter* filter, float alpha);
float SmoothFilterUpdate(SmoothFilter* filter, float input);
void AverageFilterInitialize(AverageFilter*filter);
float AverageFilterUpdate(AverageFilter*filter, float input);


float AngleLimit(float angle, float limit_min, float limit_max);
float AbsLimiter(float val, float max);
float DoubleEdgeLimiter(float val, float min, float max);
float InvSqrt(float x);
int Sign(float x);

/* 标量卡尔曼滤波器 */
void ScalarKalmanFilterInit(ScalarKalmanFilter* kf, float Q, float R, float dt);
void ScalarKalmanFilterReset(ScalarKalmanFilter* kf, float angle_deg);
float ScalarKalmanFilterUpdate(ScalarKalmanFilter* kf, float z, float u);
float ScalarKalmanUpdateAdaptive(ScalarKalmanFilter* kf, float z, float u, float chi_thresh);

void leastSquareLinearFit(leastSquareLinear* data);
#endif

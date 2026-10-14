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

/**
 * \brief 分频器：主循环每周期 Tick 一次，每 div 次返回 1
 * 
 * \note  Reset 后的下一次 Tick 立即触发，保证外环先于内环给出参考
 */
typedef struct
{
    uint16_t div;
    uint16_t count;
}RateDivider;

static inline void RateDividerInit(RateDivider* divider, uint16_t div)
{
    divider->div = div == 0U ? 1U : div;
    divider->count = 0U;
}

static inline void RateDividerReset(RateDivider* divider)
{
    divider->count = 0U;
}

static inline uint8_t RateDividerTick(RateDivider* divider)
{
    if (divider->count == 0U)
    {
        divider->count = divider->div - 1U;
        return 1U;
    }
    divider->count--;
    return 0U;
}

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

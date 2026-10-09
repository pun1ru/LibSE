#include "math.h"
#include "bsp_dwt.h"
#include "adrc.h"
#include "algorism.h"
#include "gimbalControl.h"

/**
  * @brief  fhan最速综合函数
  * @note 根据输入信号的大小和变化率自适应地调整输出信号的幅值和频率，使得输出信号能够最快地跟踪输入信号，同时抑制噪声和干扰的影响
  *
  */
extern GimbalControl gimbalControl;
static float fhan(float x1, float x2, float r, float h)
{
	float d, a0, y, a1, a2, sy, a, sa;	
	d  = r * square(h);
	a0 = h * x2;
	y  = x1 + a0;
	a1 = sqrt(d * (d + 8 * fabs(y)));
	a2 = a0 + fsgn(y) * (a1 - d) / 2.0f;
	sy = (fsgn(y + d) - fsgn(y - d)) / 2.0f;
	a = (a0 + y - a2) * sy + a2;
	sa = (fsgn(a + d) - fsgn(a - d)) / 2.0f;
	return -r * (a / (1.0f * d) - fsgn(a)) * sa - r * fsgn(a);
}

/**
  * @brief  非线性fal函数
  * @retval float 
  * @note y = e / delta^(1-alpha) when fabs(e) < fabs(delta)
  * 	  y = sign(e) * fabs(e)^alpha when fabs(e) > fabs(delta)
  */
float Fal(float e, float alpha, float delta)
{
	if (fabs(e) <= fabs(delta))
		return e / powf(fabs(delta), 1 - alpha);
	else
		return fsgn(e) * powf(fabs(e), alpha);
}//误差小采用线性部分,误差大展现幂函数特性
void LTDInitialize(LTD* ltd, float r, float h, float min, float max)
{
	ltd->r = r;
	ltd->min = min;
	ltd->max = max;
	ltd->h = h;
}

void LTDSetParam(LTD* ltd, float r, float h, float min, float max)
{
	ltd->r   = r;
	ltd->h   = h;
	ltd->min = min;
	ltd->max = max;
}

void LTD_Reset(LTD* ltd, float x1)
{
	ltd->x1 = x1;
	ltd->x2 = 0.0f;
	ltd->error_sum = 0.0f;
}
/*PS:怎么说呢，改大这玩意有时候会超大变成NAN*/
void LTDUpdate(LTD* ltd, float target)//LTD和普通TD有什么区别?高了一阶似乎，更平滑了？大概
{
	float x1_delta = ltd->x1 - target;//一阶？
	ltd->h = DWT_GetDeltaT(&ltd->cnt);//步长？
	if(ltd->min != ltd->max)
		x1_delta = AngleLimit(x1_delta, ltd->min, ltd->max);//限幅一下
	
	ltd->x1 += ltd->h * ltd->x2;//积分 
	ltd->x2 += ltd->h * (- 2  * ltd->r * ltd->x2 - ltd->r * ltd->r * x1_delta);//
	//dx2/dt = -2*r*x2 - r²*(x1 - target) 
	//将此方程看作 弹簧-阻尼系统 的动力学模型：什么牛魔玩意，那我怎么调参
	// ***** 增加安全检查 *****
  if (!isfinite(ltd->x1) || !isfinite(ltd->x2)) {
       // 数值计算发散了！进行紧急复位处理
       ltd->x1 = target; // 将状态强行拉回到目标值
       ltd->x2 = 0.0f;     // 将速度清零
       // 你还可以在这里设置一个错误标志或打印一条调试信息
    }
	if(ltd->min != ltd->max)
		ltd->x1 = AngleLimit(ltd->x1, ltd->min, ltd->max);
}

/*
 * LTDUpdateNoLimit: 与 LTDUpdate 相同的二阶跟踪微分器实现，
 * 但不对 x1 做周期性限幅（不调用 AngleLimit），适用于非周期角度
 * 或不需要环绕处理的信号。保留数值发散保护。
 */
void LTDUpdateNoLimit(LTD* ltd, float target)
{
	float x1_delta = ltd->x1 - target;
	ltd->h = DWT_GetDeltaT(&ltd->cnt);

	/* 状态更新（二阶系统积分） */
	ltd->x1 += ltd->h * ltd->x2;
	ltd->x2 += ltd->h * (-2.0f * ltd->r * ltd->x2 - ltd->r * ltd->r * x1_delta);

	/* 数值安全检查：发散时复位到目标并清速度 */
	if (!isfinite(ltd->x1) || !isfinite(ltd->x2)) {
		ltd->x1 = target;
		ltd->x2 = 0.0f;
	}

	/* 注意：此函数不做 x1 的限幅 or 环绕处理 */
}
//void LTDUpdate(LTD* ltd, float target)
//{
//    // 1. 在函数入口处，就检查输入状态是否合法
//    if (!isfinite(ltd->x1) || !isfinite(ltd->x2)) {
//        // 如果状态已经损坏，立即复位并返回
//        ltd->x1 = target;
//        ltd->x2 = 0.0f;
//        return;
//    }

//    ltd->h = DWT_GetDeltaT(&ltd->cnt);
//    float x1_delta = ltd->x1 - target;

//    if(ltd->min != ltd->max) {
//        x1_delta = AngleLimit(x1_delta, ltd->min, ltd->max);
//    }
//    
//    // 2. 将复杂的计算拆分成多个安全的步骤
//    float term1, term2, total_acceleration;

//    // 计算第一项
//    term1 = -2.0f * ltd->r * ltd->x2;
//    if (!isfinite(term1)) { // 检查中间结果
//        // 处理发散...
//        ltd->x1 = target;
//        ltd->x2 = 0.0f;
//        return;
//    }

//    // 计算第二项
//    term2 = -ltd->r * ltd->r * x1_delta;
//    if (!isfinite(term2)) { // 检查中间结果
//        // 处理发散...
//        ltd->x1 = target;
//        ltd->x2 = 0.0f;
//        return;
//    }

//    total_acceleration = term1 + term2;
//    if (!isfinite(total_acceleration)) { // 检查最终的加速度值
//        // 处理发散...
//        ltd->x1 = target;
//        ltd->x2 = 0.0f;
//        return;
//    }

//    // 3. 只有在所有计算都安全的情况下，才更新状态
//    ltd->x1 += ltd->h * ltd->x2;
//    ltd->x2 += ltd->h * total_acceleration;

//    // 4. 最终再做一次限幅
//    if(ltd->min != ltd->max) {
//        ltd->x1 = AngleLimit(ltd->x1, ltd->min, ltd->max);	
//    }
//}
void LTDPIDInitialize(LTDPID* ltdpid,float kp,float _pitchkd,float w_d_limit,float p_output_limit)
{
	ltdpid->kp=kp;
	ltdpid->_pitchkd=_pitchkd;
	ltdpid->w_d_limit=w_d_limit;
	ltdpid->p_output_limit=p_output_limit;
}

/**
 * @brief TD微分器初始化
 * @param [0]TD结构体
 * @param [1]速度因子
 * @param [2]积分步长
 * @param [3]积分步长扩大系数
 * @param [4]周期信号下限
 * @param [5]周期信号上限
 * @retval void
 * @note 若处理非周期信号，设置上下限均为0 
 */
void TDInitialize(TD* td, float r, float h0, float N, float min, float max)
{
	td->h = h0 * N;
	td->r = r;
	td->max = max;
	td->min = min;
}

void TDSetParam(TD* td, float r, float h0, float N, float min, float max)
{
	td->h = h0 * N;
	td->r = r;
	td->max = max;
	td->min = min;
}

void TD_Reset(TD* td, float x1)
{
	td->x1 = x1;
	td->x2 = 0.0f;
}

/**
 * @brief TD微分器计算
 * @param [0]TD结构体
 * @param [1]目标量
 * @retval void
 */
void TDUpdate(TD* td, float target)
{
	float x1_delta = td->x1 - target;
	
	if(td->min != td->max)
		x1_delta = AngleLimit(x1_delta, td->min, td->max);
	
	td->x1 += td->h * td->x2;
	td->x2 += td->h * fhan(x1_delta, td->x2, td->r, td->h);
	
	if(td->min != td->max)
		td->x1 = AngleLimit(td->x1, td->min, td->max);	
}

/**
 * @brief ESO扩张状态观测器初始化
 *  一般取b_01 = 1/h b_02 = 1/(3*h*h) b_03 = 1 / (20*h*h*h)
 * @param [0]ESO结构体
* @param [1]积分步长，若b取为0，则取消控制量修正，将电机控制量囊括在外部扰动中
 * @param [2]x_dot = Ax + Bu中控制矩阵u关于一阶量的因子
 * @param [3]z1_dot = h * (z2 - b_01 * error)
 * @param [4]z2_dot = h * (z3 - b_02 * Fal(error, 0.5, h) + b*u)
 * @param [5]z3_dot = -h * (b_03 * Fal(error, 0.25, h))
 * @param [6]若一阶信号为周期信号，该参数给为周期限幅下限，否则给0
 * @param [7]若一阶信号为周期信号，该参数给为周期限幅上限，否则给0
 * @retval void
 */
void ESOInitialize(ESO* eso, float h, float b, float b_01, float b_02, float b_03, float z3_limit, float min, float max)
{
	eso->h = h;
	eso->b = b;
	eso->beta_01 = b_01;
	eso->beta_02 = b_02;
	eso->beta_03 = b_03;
	eso->z3_limit = z3_limit;
	eso->max = max;
	eso->min = min;
}

/**
 * @brief ESO 扩张状态观测器更新
 * @param [0] ESO结构体
 * @param [1] 受控目标1阶返回观测值
 * @param [2] 给予控制目标先验控制量
 * @retval void
 */
void ESOUpdate(ESO* eso, float feedback, float control_val)
{
	float e = eso->z1 - feedback;//没有控制输入时eso观测会乱瞟,尝试限幅/状态机判断
	if(eso->min != eso->max)
		e = AngleLimit(e, eso->min, eso->max);
    eso->z1 += eso->h * (eso->z2 - eso->beta_01 * e);
	if(eso->b != 0)
		eso->z2 += eso->h * (eso->z3 - eso->beta_02 * Fal(e, 0.5, 1*eso->h) + eso->b * control_val);
	else
		eso->z2 += eso->h * (eso->z3 - eso->beta_02 * Fal(e, 0.5, 1*eso->h));//改成10被步长
    eso->z3 -= eso->h * (eso->beta_03 * Fal(e, 0.25, eso->h));	
   	eso->z3 = AbsLimiter(eso->z3, eso->z3_limit);
	
	if(eso->min != eso->max)
		eso->z1 = AngleLimit(eso->z1, eso->min, eso->max);
    eso->z2 = DoubleEdgeLimiter(eso->z2,eso->z2_min,eso->z2_max);
	  eso->z1 = DoubleEdgeLimiter(eso->z1,eso->z1_min,eso->z1_max);
}

/**
 * @brief ESO 状态复位：z1对齐当前值，z2/z3清零
 */
void ESO_Reset(ESO* eso, float z1)
{
	eso->z1 = z1;
	eso->z2 = 0.0f;
	eso->z3 = 0.0f;
}

//void ESOUpdate(ESO* eso, float feedback, float control_val)
//{
//	float e = eso->z1 - feedback;//没有控制输入时eso观测会乱瞟,尝试限幅/状态机判断
//	//extern float debug_w;
//	//float e_w = eso->z2 - debug_w;
//	if(eso->min != eso->max)
//		e = AngleLimit(e, eso->min, eso->max);
//    eso->z1 += eso->h * (eso->z2 - eso->beta_01 * e);
//	if(eso->b != 0)
//		eso->z2 = gimbalControl.GimbalEstimate.pitch_angular_velocity_dps;
//	else
//		eso->z2 += eso->h * (eso->z3 - eso->beta_02 * Fal(e, 0.5, eso->h));
//    eso->z3 -= eso->h * (eso->beta_03 * Fal(e, 0.25, eso->h));	
//   	eso->z3 = AbsLimiter(eso->z3, eso->z3_limit);
//	
//	if(eso->min != eso->max)
//		eso->z1 = AngleLimit(eso->z1, eso->min, eso->max);
//    eso->z2 = DoubleEdgeLimiter(eso->z2,eso->z2_min,eso->z2_max);
//	  eso->z1 = DoubleEdgeLimiter(eso->z1,eso->z1_min,eso->z1_max);	
//}

/// @brief 线性组合器初始化
/// @param esf ESF结构体
/// @param k_0 误差积分放大系数
/// @param k_1 误差放大系数
/// @param k_2 误差微分放大系数
/// @param e_0_max 误差积分上限
/// @param output_limit 输出限幅
void LESFInitialize(ESF* esf, float k_0, float k_1, float k_2, float e_0_max, float output_limit)
{
	esf->k_0 = k_0;
	esf->k_1 = k_1;
	esf->k_2 = k_2;
	esf->e_0_max = e_0_max;
	esf->output_limit = output_limit;
}

/**
 * @brief 线性组合器计算更新
 * @param [0]esf结构体
 * @param [1]一阶误差输入
 * @param [2]二阶误差输入
 * @retval output_val
 */
float LESFUpdate(ESF* esf, float e_1, float e_2)
{
	esf->e_0 += e_1;//相当于I
	esf->e_1 = e_1;
	esf->e_2 = e_2;
	esf->e_0 = AbsLimiter(esf->e_0, esf->e_0_max);
	esf->output = esf->e_0 * esf->k_0 + e_1 * esf->k_1 + e_2 * esf->k_2;
	esf->output = AbsLimiter(esf->output, esf->output_limit);
	return esf->output;
}


/// @brief LADRC结构体初始化
/// @param adrc adrc结构体
/// @param td_init_val td环节参数 td_init_val[3]={r, h0, N}
/// @param esf_init_val esf环节参数（线性esf初始化） esf_init_cal[5]={k_0, k_1, k_2, e_0_max, output_limit}
/// @param eso_init_val eso环节参数 eso_init_cal[6]={h, b, beta_01, beta_02, beta_03, z3_limit}
/// @param min 若处理的信号为周期函数，min为周期下限
/// @param max 若处理的信号为周期函数，max为周期上限，周期在adrc不同环节内统一，若为非周期信号，请输入任意min = max
/// @note 严禁修改结构体内变量顺序，给予参数请严格按照注释给定
void LADRCInitialize(ADRC* adrc, float* td_init_val, float* lesf_init_val, float* eso_init_val, float min, float max)
{
	TDInitialize(&adrc->td, td_init_val[0], td_init_val[1], td_init_val[2], min, max);
	LESFInitialize(&adrc->esf, lesf_init_val[0], lesf_init_val[1], lesf_init_val[2], lesf_init_val[3], lesf_init_val[4]);
	ESOInitialize(&adrc->eso, eso_init_val[0], eso_init_val[1], eso_init_val[2], eso_init_val[3], eso_init_val[4], eso_init_val[5], min, max);
	adrc->limit_max = max;
	adrc->limit_min = min;
}

/// @brief 线性自抗扰计算
/// @param adrc 自抗扰控制计算所需结构体
/// @param target 控制目标期望值
/// @param feedback 控制目标反馈值
void LADRCUpdate(ADRC* adrc, float target, float feedback)
{
	//input transection
	TDUpdate(&adrc->td, target);
	//calculate control value without disturbance compensation
	adrc->u_0 = LESFUpdate(&adrc->esf, AngleLimit(adrc->td.x1 - adrc->eso.z1, adrc->limit_min, adrc->limit_max), adrc->td.x2 - adrc->eso.z2);
	//estimate disturbance compensation 
	if(adrc->eso.b == 0)
		adrc->u = adrc->u_0;
	else
		adrc->u = (adrc->u_0 - adrc->eso.z3) / (adrc->eso.b * 1.0f);
	//update extensional state observer
	ESOUpdate(&adrc->eso, feedback, adrc->u);
}

/*
 * Yaw observer retained from 47c7530. Pitch uses the upper-board ESOUpdate()
 * path with explicit z1/z2 bounds, so the two tuned observers stay isolated.
 */
static void YawTunedESOUpdate(ESO* eso, float feedback, float control_val)
{
	float e = eso->z1 - feedback;

	if (eso->min != eso->max)
		e = AngleLimit(e, eso->min, eso->max);

	eso->z1 += eso->h * (eso->z2 - eso->beta_01 * e);
	eso->z2 += eso->h * (eso->z3
		- eso->beta_02 * Fal(e, 0.5f, 10.0f * eso->h)
		+ eso->b * control_val);
	eso->z3 -= eso->h * eso->beta_03 * Fal(e, 0.25f, eso->h);
	eso->z3 = AbsLimiter(eso->z3, eso->z3_limit);

	if (eso->min != eso->max)
		eso->z1 = AngleLimit(eso->z1, eso->min, eso->max);
}

void YawLADRCUpdate(ADRC* adrc, float target, float feedback)
{
	TDUpdate(&adrc->td, target);
	adrc->u_0 = LESFUpdate(
		&adrc->esf,
		AngleLimit(adrc->td.x1 - adrc->eso.z1, adrc->limit_min, adrc->limit_max),
		adrc->td.x2 - adrc->eso.z2);

	if (adrc->eso.b == 0.0f)
		adrc->u = adrc->u_0;
	else
		adrc->u = (adrc->u_0 - adrc->eso.z3) / adrc->eso.b;

	YawTunedESOUpdate(&adrc->eso, feedback, adrc->u);
}

/// @brief 线性自抗扰计算 V2 — e2 使用实际测量速度代替 ESO.z2
/// @param adrc 自抗扰控制计算所需结构体
/// @param target 控制目标期望值 (rad)
/// @param feedback 控制目标反馈值 (rad)
/// @param actual_velocity 实际测量角速度 (rad/s)，替代 eso.z2 参与 LESF 组合
/// @param z3_gain 扰动补偿权重 (0~1)，乘在 z3 上：u = (u0 - z3_gain*z3) / b
void LADRCUpdateV2(ADRC* adrc, float target, float feedback, float actual_velocity, float z3_gain)
{
	TDUpdate(&adrc->td, target);
	/* V2: e2 = td.x2 - actual_velocity，用实测速度替换 ESO 估计速度 */
	adrc->u_0 = LESFUpdate(&adrc->esf,
		AngleLimit(adrc->td.x1 - adrc->eso.z1, adrc->limit_min, adrc->limit_max),
		adrc->td.x2 - actual_velocity);
	if(adrc->eso.b == 0)
		adrc->u = adrc->u_0;
	else
		adrc->u = (adrc->u_0 - z3_gain * adrc->eso.z3) / (adrc->eso.b * 1.0f);
	ESOUpdate(&adrc->eso, feedback, adrc->u);
}

void LTDADRCUpdate(ADRC* adrc,LTD* ltd, float target, float feedback)
{
	//input transection
	LTDUpdate(ltd, target);//LTD给出期望目标位置,速度
	//calculate control value without disturbance compensation//强行改一下哎试试gimbalControl.GimbalEstimate.pitch_angular_velocity_dps
	//居然挺有用的z2换成真是角速度
//	adrc->u_0 = LESFUpdate(&adrc->esf, AngleLimit(ltd->x1 - adrc->eso.z1, adrc->limit_min, adrc->limit_max), ltd->x2 - adrc->eso.z2);
	adrc->u_0 = LESFUpdate(&adrc->esf, AngleLimit(ltd->x1 - adrc->eso.z1, adrc->limit_min, adrc->limit_max), ltd->x2 - gimbalControl.GimbalEstimate.pitch_angular_velocity_dps)+ltd->x2*0;//加速度前馈试试
	//estimate disturbance compensation //更新一阶e1,二阶e2,计算应该给出的控制量
	if(adrc->eso.b == 0)
		adrc->u = adrc->u_0;
	else
		adrc->u = (adrc->u_0 - adrc->eso.z3) / (adrc->eso.b * 1.0f);//最终给出的控制量就是lsef给出的控制量-观测的扰动再除以控制增益
	//update extensional state observer
	ESOUpdate(&adrc->eso, feedback, adrc->u);//eso更新观测的位置和速度
}

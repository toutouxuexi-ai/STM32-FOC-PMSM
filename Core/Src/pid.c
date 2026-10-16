#include "pid.h"
#include "foc.h"
/* D/Q轴电流环PI实例(表贴电机Ld=Lq, 两轴参数一致) */
Current_PID_DEF Current_D_PID;
Current_PID_DEF Current_Q_PID;
Current_PID_DEF Speed_PID;
Current_PID_DEF Angle_PID;
/* ==================== PID初始化 ====================
 * 本组参数为带电机实调整定, 实测电流跟踪效果良好, 以实测效果为准:
 * Kp=1, Ki=0.01(离散值, 已含Ts; TIM1中心对齐PWM=10kHz, Ts=0.1ms)
 * 理论参考(电机规格书42JSF840AS-1000-8新款: 相电阻R=0.59Ω, 相电感L=0.66mH):
 * 零极对消 Ki/Kp=R/L≈894/s → Ki理论≈0.089, 但实际受采样延迟/死区/
 * 电感口径等影响, 理论值仅作起步参考, 不替代实测整定
 * 输出限幅±4V → 堵转电流4V/0.59Ω≈6.8A, 低于硬件过流阈值10.54A */
void PID_Init(void)
{
	Current_D_PID.Kp = 1.0f;
	Current_D_PID.Ki = 0.001f;   /* 与Q轴一致; 无积分id会有静差, 抗不住ωL耦合 */
	Current_D_PID.Kd = 0.0f;
	Current_D_PID.Up_limit = 4.0f;
	Current_D_PID.Down_limit = -4.0f;
Current_D_PID.I_sum = 0.0f;

	Current_Q_PID.Kp = 0.8f;
	Current_Q_PID.Ki = 0.01f;
	Current_Q_PID.Kd = 0.0f;
	Current_Q_PID.Up_limit = 12.0f;
	Current_Q_PID.Down_limit = -12.0f;
	Current_Q_PID.I_sum = 0.0f;
	
	Speed_PID.Kp = 0.005f;
	Speed_PID.Ki = 0.0011f;
	Speed_PID.Kd = 0.8f;
	Speed_PID.Up_limit = 5.9f;
	Speed_PID.Down_limit = -5.9f;   
	Speed_PID.I_sum = 0.0f;
	
	Angle_PID.Kp = 0.2f;
	Angle_PID.Ki = 0.00f;
	Angle_PID.Kd = 0.0f;
	Angle_PID.Up_limit = 100.0f;
	Angle_PID.Down_limit = -100.0f;   
	Angle_PID.I_sum = 0.0f;
	
	
}

/* ==================== PID计算 ====================
 * 功  能: 位置式PI, 带积分限幅+输出限幅(抗积分饱和)
 * 形  参: ref-给定  input-反馈  output-输出(Vd或Vq)  current_pid-PI实例
 * 说  明: 每个PWM周期调用一次; 误差取ref-input
 *         (正方向已实测验证: Vq↑→Iq↑) */
void Current_PID_Calc(float ref, float input, float* output, Current_PID_DEF* current_pid)
{
	/* 计算误差 */
	current_pid->err = ref - input;

	/* 比例项 */
	current_pid->err_Kp = current_pid->Kp * current_pid->err;

	/* 积分项(系数已含Ts), 积分先限幅抗饱和 */
	current_pid->I_sum += current_pid->Ki * current_pid->err;
	
	//current_pid->I_sum - (* output) * Speed_PID.Kd
	if(current_pid->I_sum > current_pid->Up_limit)
		current_pid->I_sum = current_pid->Up_limit;
	if(current_pid->I_sum < current_pid->Down_limit)
		current_pid->I_sum = current_pid->Down_limit;

	/* 合计输出并限幅 */
	*output = current_pid->err_Kp + current_pid->I_sum;
	if(*output > current_pid->Up_limit)
		*output = current_pid->Up_limit;
	if(*output < current_pid->Down_limit)
		*output = current_pid->Down_limit;
}

void Speed_PID_Calc(float ref, float input, float* output, Current_PID_DEF* Speed_PID)
{
	Speed_PID->err = ref - input;
	Speed_PID->err_Kp = Speed_PID->Kp * Speed_PID->err;

	/* 抗积分饱和: 输出已顶到限幅且误差同向时, 积分不充电 */
	float out_try = Speed_PID->err_Kp + Speed_PID->I_sum;
	if (!((out_try >= Speed_PID->Up_limit   && Speed_PID->err > 0) ||
	      (out_try <= Speed_PID->Down_limit && Speed_PID->err < 0)))
	{
		Speed_PID->I_sum += Speed_PID->Ki * Speed_PID->err;
	}

	if(Speed_PID->I_sum > Speed_PID->Up_limit)
		Speed_PID->I_sum = Speed_PID->Up_limit;
	if(Speed_PID->I_sum < Speed_PID->Down_limit)
		Speed_PID->I_sum = Speed_PID->Down_limit;

	*output = Speed_PID->err_Kp + Speed_PID->I_sum;
	if(*output > Speed_PID->Up_limit)   *output = Speed_PID->Up_limit;
	if(*output < Speed_PID->Down_limit) *output = Speed_PID->Down_limit;
}

void position_PID_Calc(float ref, float input, float* output, Current_PID_DEF* angle_PID)
{
	angle_PID->err = ref - input;
	angle_PID->err_Kp = angle_PID->Kp * angle_PID->err;

	/* 抗积分饱和: 输出已顶到限幅且误差同向时, 积分不充电 */
	float out_try = angle_PID->err_Kp + angle_PID->I_sum;
	if (!((out_try >= angle_PID->Up_limit   && angle_PID->err > 0) ||
	      (out_try <= angle_PID->Down_limit && angle_PID->err < 0)))
	{
		angle_PID->I_sum += angle_PID->Ki * angle_PID->err;
	}
	if(angle_PID->I_sum > angle_PID->Up_limit)
		angle_PID->I_sum = angle_PID->Up_limit;
	if(angle_PID->I_sum < angle_PID->Down_limit)
		angle_PID->I_sum = angle_PID->Down_limit;

	*output = angle_PID->err_Kp + angle_PID->I_sum;
	if(*output > angle_PID->Up_limit)   *output = angle_PID->Up_limit;
	if(*output < angle_PID->Down_limit) *output = angle_PID->Down_limit;
}

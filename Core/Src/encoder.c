#include  "encoder.h"
#include  "foc.h"
#include "tim.h"
#include "adc.h"
#include "pid.h"
#include "if_drag.h"
#include "smo.h"
Encoder_Typedef Encoder;



void Encoder_Init(void)
{
	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
	Encoder.Z =0;
	Encoder_ZeroAlignment();
	HAL_ADCEx_InjectedStart_IT(&hadc1); 
}

void Encoder_ZeroAlignment(void)
{
	/* d轴电压: 转子N极被吸到theta=0方向; 用FOC_Init里已设好的Tpwm=8400, 不要改 */
	Voltage_DQ.Vd = 1.5f;
	Voltage_DQ.Vq = 0.0f;
	
	rtX.Udc = 24;
	rtX.Tpwm = 8400;


	/* 关键: 停掉FOC中断, 否则Motor_Run每个PWM周期用旋转角度覆盖CCR, 静态场保不住 */
	HAL_ADCEx_InjectedStop_IT(&hadc1);

	Angle_To_Cos_Sin(0.0f, &Transf_Cos_Sin);                            /* 静止磁场, 电角度0 */
	Rev_Park_Transf(Voltage_DQ, Transf_Cos_Sin, &Voltage_Alpha_Beta);   /* RevPark变换 */
	SVPWM_Calc(Voltage_Alpha_Beta, rtX.Udc, rtX.Tpwm);                  /* SVPWM调制 */
	TIM1->CCR1 = rtY.Tcmp1;   /* 写一次即可, CCR会一直保持, 不需要循环刷新 */
	TIM1->CCR2 = rtY.Tcmp2;
	TIM1->CCR3 = rtY.Tcmp3;

	HAL_Delay(2000);          /* 保持1s等转子吸稳, 此时扭转子应有4个吸点(4对极) */
	TIM3->CNT = 0;            /* 吸住的位置 = 编码器零点 */

}

//void Encoder_SpeedCalc(void)
//{
//    static uint16_t cnt_last = 0;
//    static uint8_t  first = 1;          // 防止上电第一拍算出假速度

//    uint16_t cnt = (uint16_t)TIM3->CNT;

//    if (first) { cnt_last = cnt; first = 0; }

//    int16_t d = (int16_t)(cnt - cnt_last);   // 回绕在这里自动抵消，相当于先uint在int
//    cnt_last = cnt;

//   
//	//Encoder.angle += (float)d * (2.0f * PI / 1000.0f);
//	Encoder.speed = (float)d * (2.0f * PI / 1000.0f) / 0.001f;  //得到电角速度
//	
//	if(Encoder.angle > 2*PI)
//		Encoder.angle -= 2*PI;
//	else if(Encoder.angle < 0)
//		Encoder.angle += 2*PI;
//}
void Encoder_SpeedCalc(void)
{
    static uint16_t cnt_last = 0;
    static uint8_t  first = 1, div = 0;
    static int32_t  d_acc = 0;

    uint16_t cnt = (uint16_t)TIM3->CNT;
    if (first) { cnt_last = cnt; first = 0; }
    int16_t d = (int16_t)(cnt - cnt_last);
    cnt_last = cnt;

    d_acc += d;                       /* 累计2ms的计数 */
    if (++div >= 2) {
        Encoder.speed = (float)d_acc * (2.0f*PI/1000.0f) / 0.002f;
        div = 0; d_acc = 0;
    }
}


/* 给定斜坡: 每1ms最多变5 rad/s (跨200用40ms, 想快慢改这个数) */
#define RAMP_STEP  10.0f
static float speed_ramp = 0.0f;         /* 斜坡后的实际给定 */

void SysTick_ISR_encoder(void)
{
    if      (rtX.speed_ref > speed_ramp + RAMP_STEP) speed_ramp += RAMP_STEP;   /* 目标还高很多: 涨5 */
    else if (rtX.speed_ref < speed_ramp - RAMP_STEP) speed_ramp -= RAMP_STEP;   /* 目标还低很多: 降5 */
    else                                             speed_ramp  = rtX.speed_ref; /* 快到了: 直接跳过去 */

    Encoder_SpeedCalc();
    Encoder.speed_filter = 0.40f * Encoder.speed + 0.60f * Encoder.speed_filter;
	
	
	
	static uint8_t ok_cnt = 0;

if (s_state == IF_RUNNING)
{
    if (fabsf(SMO_OUTPUT.speed_pll - s_speed_if) < 15.0f)
    {
        if (++ok_cnt >= 50)
        {
            s_state = IF_IDLE;
            speed_ramp     = 400.0f;   // ① 速度给定对齐当前速度
            //Speed_PID.I_sum = 2.0f;    // ② 积分预载当前Iq
			s_state = IF_IDLE;
speed_ramp      = s_speed_if;    /* 原来写400, 改成对齐当前速度=500 */
Speed_PID.I_sum = 2.0f;
/* 相位过渡初始化 */
trans_offset = s_theta_if - SMO_OUTPUT.theta_pll;
while (trans_offset >  PI) trans_offset -= 2.0f*PI;
while (trans_offset < -PI) trans_offset += 2.0f*PI;
trans_cnt = 0;

        }
    }
    else
    {
        ok_cnt = 0;
    }
}
if (s_state== IF_IDLE)
	Speed_PID_Calc(speed_ramp, SMO_OUTPUT.speed_pll, &Current_ref.Iq, &Speed_PID);
	
	//position_PID_Calc(rtX.angle_ref, Encoder.angle_mach, &rtX.speed_ref, &Angle_PID);
    //Speed_PID_Calc(speed_ramp, Encoder.speed_filter, &Current_ref.Iq, &Speed_PID);  /* 用speed_ramp */
}

//void SysTick_ISR_encoder(void)
//{
//	
//	Encoder_SpeedCalc();
//	//Encoder_SpeedFilter();
//	Encoder.speed_filter = 0.40f * Encoder.speed + 0.60f * Encoder.speed_filter;
//	Speed_PID_Calc(rtX.speed_ref, Encoder.speed_filter, &Current_ref.Iq, &Speed_PID);
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(Encoder.Z==0)
		Encoder.Z=1;
	else if(Encoder.Z==1)
		Encoder.Z=0;
}

static void Encoder_SpeedFilter(void)
{
    static uint8_t i;
    static float speed_mean, speed[10];

    speed[i++] = Encoder.speed;
    if(i == 10)
    {
        i = 0;
        speed_mean = 0;
        BubbleSort(&speed[0], 10);
        for(uint8_t j=2; j<8; j++)
        {
            speed_mean += speed[j];
        }
        speed_mean /= 6.0f;
        Encoder.speed_filter = speed_mean;   /* 老师架构: 速度恒为正, 无符号处理 */
		
       //Speed_PID_Calc(rtX.speed_ref, Encoder.speed_filter, &Current_ref.Iq, &Speed_PID);
		
    }
}

// 冒泡排序（升序）
void BubbleSort(float* speed, uint8_t length)
{
    for (int i = 0; i < length - 1; i++)
    {
        for (int j = 0; j < length - 1 - i; j++)
        {
            if (speed[j] > speed[j + 1])
            {
                float temp = speed[j];
                speed[j] = speed[j + 1];
                speed[j + 1] = temp;
            }
        }
    }
}

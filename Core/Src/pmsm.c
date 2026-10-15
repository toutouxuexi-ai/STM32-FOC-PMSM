#include "pmsm.h" // Device header  
#include "adc.h"
#include "math.h"
#include "tim.h"
#include "foc.h"
#include "hall.h"
#include "encoder.h"
float I_uvw[3];
float power;
float  temper;
float  Rt,temp;

float cnt;
void Motor_Init(void)
{
	//pwm通道和互补通道开启
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//adc注入和规则开启	
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_DMABuff, 2);
//这里的中断使能放到校准的后面才打开了
HAL_ADCEx_InjectedStart(&hadc2);
HAL_ADCEx_InjectedStart(&hadc3);


}


void Motor_Run(void)
{
	static float theta;
	

	Hall.angle_inc = Hall.speed * 0.0001f;
	
	
	Hall.angle += Hall.angle_inc;
	
	

	if(Hall.angle > 2*PI)
		Hall.angle -= 2*PI;
	if(Hall.angle < 0.0f)
		Hall.angle += 2*PI;
	


if(cnt<10000)
	{
	//cnt++;	                     /* 开环拖动1s后切闭环; 注释掉会永远开环 */
//		if(rtX.speed_ref > 0){theta += 0.005f;}
//	else if(rtX.speed_ref < 0){theta -= 0.005f;}
//	
//		if(theta > 2*PI)
//		theta -= 2*PI;
//	else if(theta < 0)
//		theta += 2*PI;
	
	rtX.theta=Encoder.angle;
	}
	
else{
		
	rtX.theta = Hall.angle;
	//rtX.theta = Hall.angle;
	
	}
	
	/* 采样电路: Iout = 6*0.02Ω*I + 1.25V, 灵敏度0.12V/A, 偏置1.25V
	   下桥采样方向与电机惯例相反, 整体取反使 正电流=流入电机
	   注意: U/V 相与 ADC1/ADC2 的对应关系以实际接线为准 */
	
	power =  (ADC_DMABuff[1]*3.3f/4096.0f)*25.0f;
	rtX.Udc = power;
	
	
	
	
	
	FOC_Run();

	/* PWM输出(此前整段丢失, 已恢复); 老师形式: 无相序交换, 直通 */
	TIM1->CCR1 = rtY.Tcmp1;
	TIM1->CCR2 = rtY.Tcmp2;
	TIM1->CCR3 = rtY.Tcmp3;
}

void Motor_GetTemperature(void)
{
	
	 Rt = 3.3f * 4700.0f / (ADC_DMABuff[0] * 3.3f / 4096.0f ) - 4700.0f;
	 temp = Rt / 10000.0f;
	 temp = log(temp); 
	 temp /= 3380.0f; 
	 temp += (1.0f /  (273.15f + 25.0f)); 
	 temp = 1.0f / (temp); 
	 temp -= 273.15f; 
	 temper = temp;
}


#include "hall.h"
#include "tim.h"
#include "foc.h"
#include "pid.h"
#include "pmsm.h"

/* 软件超时判停配置:
 * HALL_TIMEOUT_MS  : 多少毫秒没有霍尔边沿就认为超时(对应最低可测转速)
 * STALL_CONFIRM_NUM: 连续多少次超时才正式判定停转, 抗干扰防误报 */
#define HALL_TIMEOUT_MS    100u
#define STALL_CONFIRM_NUM    2u

/* CCW方向翻转开关: 表里永远放纯标定物理角, 只在CCW赋值时按此开关决定是否+π
 * 置1: 控制帧翻转180°, 恒正Iq在CCW中产生反转方向转矩(磁场滞后90°)
 * 置0: 直接用标定值(仅开环拖动或带符号Iq时可用)
 * 加π自动回绕到[0,2π) */
#define CCW_FLIP_EN  0
#if CCW_FLIP_EN
#define CCW_A(a)  ((a) >= PI ? (a) - PI : (a) + PI)
#else
#define CCW_A(a)  (a)
#endif

Hall_Typedef Hall;

void Hall_Init(void)
{
	Hall.cut = 0;
	Hall.first_edge = 1u;   /* 上电第一个边沿没有上一角度基准, 丢弃 */
	__HAL_TIM_ENABLE_IT(&htim5,TIM_IT_TRIGGER);
	HAL_TIMEx_HallSensor_Start_IT(&htim5);
	
	
}

void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{
	float angle_prev;   /* if分支执行前的角度, 用于判断本次转换是否有效 */
	/* 喂狗: 来了霍尔边沿, 超时秒表和确认计数归零, 并解除停转标志 */
	Hall.edge_timeout = 0u;
	Hall.stall_cnt    = 0u;
	Hall.stall        = 0u;

	/* 霍尔模式: 硬件在边沿时复位CNT并把计数捕获到CCR1(1MHz, 1tick=1us) */
	Hall.cut += TIM5->CCR1;
	/* 每个霍尔边沿对应 π/3 电角度(一个电气周期6个边沿) */
	Hall.state_last = Hall.state;
    Hall.state  = HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10);
    Hall.state |= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11) << 1;
    Hall.state |= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12) << 2;
	
//	switch(Hall.state)
//	{
//		case 6:
//			Hall.angle = 5.70848;4.92395;//4.0066f;
//			Hall.speed = 2*PI/(Hall.cut*0.000001f);
//			Hall.cut = 0;
//			break;
//		case 2:
//			Hall.angle = 4.00956;//2.91575f;
//			break;
//		case 3:
//			Hall.angle = 2.91864;//2.06664f;
//			break;
//		case 1:
//			Hall.angle = 1.81270;//0.82562f;
//			break;
//		case 5:
//			Hall.angle = 0.86766;//6.00651f;
//			break;
//		case 4:
//			Hall.angle = 6.05687;//5.02991f;
//			break;
//	}
	
	
	/* 角度表: 两列均为标定物理角, CCW列沿反转方向递减(配pmsm.c带符号插补)
	   CCW赋值经CCW_A()宏, 按CCW_FLIP_EN决定是否+π(方向翻转), 标定值本身不动
	   每个分支只更新角度, 测速统一放到最后算 */
	angle_prev = Hall.angle;

	if(Hall.state_last == 6)
	{
		if(Hall.state == 4)      Hall.angle = 6.05687f;        /* CW */
		else if(Hall.state == 2) Hall.angle = CCW_A(4.32434f); /* CCW标定角 */
	}
	if(Hall.state_last == 4)
	{
		if(Hall.state == 5)      Hall.angle = 0.86766f;
		else if(Hall.state == 6) Hall.angle = CCW_A(5.41937f);
	}
	if(Hall.state_last == 5)
	{
		if(Hall.state == 1)      Hall.angle = 1.81270f;
		else if(Hall.state == 4) Hall.angle = CCW_A(0.27429f);
	}
	if(Hall.state_last == 1)
	{
		if(Hall.state == 3)      Hall.angle = 2.91864f;
		else if(Hall.state == 5) Hall.angle = CCW_A(1.16929f);
	}
	if(Hall.state_last == 3)
	{
		if(Hall.state == 2)      Hall.angle = 4.00956f;
		else if(Hall.state == 1) Hall.angle = CCW_A(2.28429f);
	}
	if(Hall.state_last == 2)
	{
		if(Hall.state == 6)      Hall.angle = 4.92395f;
		else if(Hall.state == 3) Hall.angle = CCW_A(3.40932f);
	}

	/* ==== 统一测速: 刚走完扇区的真实宽度 ÷ 走它花的时间 ====
	   da = 本边沿角度 - 上一有效边沿角度 = 刚走完扇区的真实跨度
	   (霍尔安装不均匀, 各扇区实际52°~65°, 比固定π/3准确;
	    da自带符号: CW为正, CCW为负, 连正负号判断都省了) */
	if(Hall.angle != angle_prev)            /* 有效转换: 角度被更新 */
	{
		float da = Hall.angle - Hall.angle_last;
		if(da >  PI) da -= 2*PI;            /* 回绕到(-π,π), 扇区小于π够用 */
		if(da < -PI) da += 2*PI;
		if(Hall.first_edge == 0u)           /* 首边沿无上一角度, 丢弃 */
			Hall.speed = da / (Hall.cut * 0.000001f);   /* cut: 1tick=1µs */
		Hall.angle_last = Hall.angle;
		Hall.first_edge = 0u;
		Hall.cut = 0u;                      /* 有效边沿才清零 */
	}
	/* 无效转换(如弹跳产生的非法跳变): 角度没更新, 不算速度,
	   cut保留累积到下个有效边沿——此时da和cut同时跨多个扇区, 宽度和时间仍然配对 */
	
		
}


// 滴答定时器中断服务函数(1ms周期): 软件超时计时 + 停转判定
void SysTick_ISR(void)
{
	/* 1ms 没边沿就加 1; 每数满一个超时窗口就把确认计数加 1,
	   秒表归零重新数下一个窗口; 连续 STALL_CONFIRM_NUM 个窗口
	   都没有边沿才正式判定, 可抗住一次偶发的信号丢失 */
	/* 只在闭环阶段(cnt>=10000)武装停转引信, 开环拖动期不判停, 避免误杀起动 */
	if(cnt >10000)
	{
		if(++Hall.edge_timeout >= HALL_TIMEOUT_MS)
		{
			Hall.edge_timeout = 0u;
			if(Hall.stall_cnt < 0xFFu)
				Hall.stall_cnt++;
			if(Hall.stall_cnt >=10 )
				Hall.stall = 1u;                    /* 正式判定停转/堵转 *///STALL_CONFIRM_NUM
		}
	}

	if(Hall.stall)
	{
		/* 停转: 速度(含滤波输出)归零, 清速度环积分, 撤掉转矩电流给定,
		   避免电机堵转期间持续通电发热; 正规项目此处应由电机状态机
		   切到 STOP/FAULT 并关断 PWM 主输出(MOE) */
		Hall.speed        = 0.0f;
		Hall.speed_filter = 0.0f;
		Hall.cut          = 0u;
		Speed_PID.I_sum   = 0.0f;
		Current_ref.Iq    = 0.0f;
	}
	else
	{
		//Hall_SpeedFilter();   /* 正常运行才做速度滤波和速度环计算 */
	}
	
}

// 霍尔速度滤波函数
//static void Hall_SpeedFilter(void)
//{
//    static uint8_t i;
//    static float speed_mean, speed[10];

//    speed[i++] = Hall.speed;
//    if(i == 10)
//    {
//        i = 0;
//        speed_mean = 0;
//        BubbleSort(&speed[0], 10);
//        for(uint8_t j=2; j<8; j++)
//        {
//            speed_mean += speed[j];
//        }
//        speed_mean /= 6.0f;
//        Hall.speed_filter = speed_mean;   /* 老师架构: 速度恒为正, 无符号处理 */
//		
//      if(cnt>=10000) Speed_PID_Calc(rtX.speed_ref, Hall.speed_filter, &Current_ref.Iq, &Speed_PID);
//		
//    }
//}

//// 冒泡排序（升序）
//void BubbleSort(float* speed, uint8_t length)
//{
//    for (int i = 0; i < length - 1; i++)
//    {
//        for (int j = 0; j < length - 1 - i; j++)
//        {
//            if (speed[j] > speed[j + 1])
//            {
//                float temp = speed[j];
//                speed[j] = speed[j + 1];
//                speed[j + 1] = temp;
//            }
//        }
//    }
//}

#include "foc.h"
#include "pmsm.h"
#include "pid.h"    /* 电流环PI模块 */
#include "encoder.h"
#include "adc.h"    /* ADC偏置校准用 */

/* ==================== 全局变量定义 ==================== */
ExtX rtX;                              /* 输入结构体变量 */
ExtY rtY;                              /* 输出结构体变量 */
Transf_Cos_Sin_DEF Transf_Cos_Sin;     /* 当前角度计算三角函数 */
Voltage_DQ_DEF Voltage_DQ;             /* DQ电压输出 */
Voltage_Alpha_Beta_DEF Voltage_Alpha_Beta;  /* RevPark变换输出 */
Current_ABC_DEF Current_Abc;                 /* 三相电流 */
Current_DQ_DEF Current_dq;                   /* DQ轴电流 */
Current_Alpha_Beta_DEF Current_Alpbet;       /* Alpha Beta电流 */

Current_DQ_DEF Current_ref;             /* DQ轴电流给定(闭环) */
ADC_Offset_DEF ADC_Offset;              /* ADC零流偏置(上电标定) */

/* ==================== ADC偏置校准 ====================
 * 功  能: 零电流时实测三相采样偏置, 替代写死的1.25V
 * 调用点: Motor_Init()之后、Encoder_Init()之前
 *         (需要TIM1已启动提供CC4注入触发; 此时CCR1-3=0,
 *          三相无差压, 电流恒为0)
 * 说  明: 注入组为multi-mode, ADC1是主机, 触发后ADC2/3
 *         自动跟随转换, 各读各的JDR1即可; 校准失败或结果
 *         不合理时退化为默认1.25V(旧行为), 不会失控
 */
void ADC_OffsetCalib(void)
{
    uint32_t s1 = 0, s2 = 0, s3 = 0;
    float u, v, w;

    /* 安全默认: 校准异常时保持旧行为 */
    ADC_Offset.u = 1.25f;
    ADC_Offset.v = 1.25f;
    ADC_Offset.w = 1.25f;

    HAL_ADCEx_InjectedStart(&hadc1);   /* 主机启动, 从机Motor_Init里已启动 */

    for (uint16_t i = 0; i < 1000; i++)
    {
        if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 10) != HAL_OK)
            return;                    /* 触发异常: 用默认偏置 */
        s1 += ADC1->JDR1;
        s2 += ADC2->JDR1;
        s3 += ADC3->JDR1;
    }

    HAL_ADCEx_InjectedStop(&hadc1);

    u = s1 * 3.3f / 4096.0f / 1000.0f;
    v = s2 * 3.3f / 4096.0f / 1000.0f;
    w = s3 * 3.3f / 4096.0f / 1000.0f;

    /* 合理性检查: 偏置应在1.25V附近(0.5~2.0V), 否则维持默认 */
    if (u > 0.5f && u < 2.0f &&
        v > 0.5f && v < 2.0f &&
        w > 0.5f && w < 2.0f)
    {
        ADC_Offset.u = u;
        ADC_Offset.v = v;
        ADC_Offset.w = w;
    }
}

/* ==================== 内部函数声明 ==================== */
void Angle_To_Cos_Sin(float angle, Transf_Cos_Sin_DEF* cos_sin);
void Rev_Park_Transf(Voltage_DQ_DEF v_dq, Transf_Cos_Sin_DEF cos_sin, Voltage_Alpha_Beta_DEF* v_alpha_beta);
void SVPWM_Calc(Voltage_Alpha_Beta_DEF v_alpha_beta, float Udc, float Tpwm);

/* ==================== FOC初始化 ====================
 * 功  能：FOC算法初始化
 * 形  参：无
 * 说  明：初始化PI控制器参数
 */
void FOC_Init(void)
{
	rtX.Udc = 24;
	rtX.Tpwm = 8400;
	rtX.Dir = CCW;

	Voltage_DQ.Vd = 0.0f;
	Voltage_DQ.Vq = 1.0f;   /* 闭环模式: Vd/Vq由电流环PI每周期计算覆盖 */

	Current_ref.Id = 0.0f;
	Current_ref.Iq = 1.0f;  /* Iq给定1A起步; 若堵转失步再往上加, 转稳后往下试 */

	rtX.speed_ref = 1256.0f;  
	PID_Init();            
}

/* ==================== FOC主运行函数 ====================
 * 功  能：FOC算法
 * 形  参：无
 * 说  明：执行FOC控制流程，用PI控制器控制电流环
 */
void FOC_Run(void)
{
	
	static uint16_t cnt_last = 0;
    static uint8_t  first = 1;          // 防止上电第一拍算出假速度

    uint16_t cnt = (uint16_t)TIM3->CNT;

    if (first) { cnt_last = cnt; first = 0; }

    int16_t d = (int16_t)(cnt - cnt_last);   // 回绕在这里自动抵消，相当于先uint在int
    cnt_last = cnt;
	Encoder.angle += (float)d * (2.0f * PI / 1000.0f);
	
	if(Encoder.angle > 2*PI)
		Encoder.angle -= 2*PI;
	else if(Encoder.angle < 0)
		Encoder.angle += 2*PI;
	
	/* 用上电实测偏置替代写死的1.25V, 消除采样直流分量导致的1×电频率转矩纹波 */
	Current_Abc.Ia = (ADC_Offset.u - ADC2->JDR1*3.3f/4096.0f)/0.12f;
	Current_Abc.Ib = (ADC_Offset.v - ADC1->JDR1*3.3f/4096.0f)/0.12f;   /* 老师形式: 无采样互换 */
	Current_Abc.Ic = (ADC_Offset.w - ADC3->JDR1*3.3f/4096.0f)/0.12f;
	/* 回退用这三行:
	Current_Abc.Ia = (1.25f - ADC2->JDR1*3.3f/4096.0f)/0.12f;
	Current_Abc.Ib = (1.25f - ADC1->JDR1*3.3f/4096.0f)/0.12f;
	Current_Abc.Ic = (1.25f - ADC3->JDR1*3.3f/4096.0f)/0.12f; */
//	Current_Abc.Ia = I_uvw[0];
//	Current_Abc.Ib = I_uvw[1];
//	Current_Abc.Ic = I_uvw[2];
	Angle_To_Cos_Sin(rtX.theta, &Transf_Cos_Sin);
	
	Clarke_Transf( Current_Abc, &Current_Alpbet);
	 Park_Transf(Current_Alpbet , Transf_Cos_Sin , &Current_dq);

	/* 电流环: D/Q轴各一个PI, 误差ref-fb -> Vd/Vq (此前被注释, 已恢复) */
	Current_PID_Calc(Current_ref.Id, Current_dq.Id, &Voltage_DQ.Vd, &Current_D_PID);
	Current_PID_Calc(Current_ref.Iq, Current_dq.Iq, &Voltage_DQ.Vq, &Current_Q_PID);

                    /* 计算三角函数 */
    Rev_Park_Transf(Voltage_DQ, Transf_Cos_Sin, &Voltage_Alpha_Beta);  /* RevPark变换 */
    SVPWM_Calc(Voltage_Alpha_Beta, rtX.Udc, rtX.Tpwm);             /* SVPWM调制 */
}

/* ==================== 角度转三角函数 ====================
 * 功  能：COS SIN值计算
 * 形  参：角度以及COS SIN结构体
 * 说  明：COS SIN值计算
 */
void Angle_To_Cos_Sin(float angle, Transf_Cos_Sin_DEF* cos_sin)
{
    cos_sin->Cos = cosf(angle);
    cos_sin->Sin = sinf(angle);
}

/* ==================== 反Park变换 ====================
 * 功  能：反PARK变换
 * 形  参：DQ轴电压、COS SIN值、alpha beta电压
 * 说  明：直流变交流
 *  Valpha = Vd * cos(theta) - Vq * sin(theta);
 *  Vbeta = Vd * sin(theta) + Vq * cos(theta);
 */
void Rev_Park_Transf(Voltage_DQ_DEF v_dq, Transf_Cos_Sin_DEF cos_sin, Voltage_Alpha_Beta_DEF* v_alpha_beta)
{
    v_alpha_beta->Valpha = cos_sin.Cos * v_dq.Vd - cos_sin.Sin * v_dq.Vq;
    v_alpha_beta->Vbeta = cos_sin.Sin * v_dq.Vd + cos_sin.Cos * v_dq.Vq;
}

/* ==================== SVPWM调制计算 ====================
 * 功  能：SVPWM计算
 * 形  参：alpha beta电压以及母线电压、定时器周期
 * 说  明：老师的经典实现：扇区由符号判断(N=4C+2B+A)，
 *         X/Y/Z线性公式求矢量作用时间，全程无 atan2/sin。
 *         相对老师原码做了两处适配(输出已与原SimpleFOC版
 *         SVPWM全扇区数值比对一致)：
 *         1) 老师公式按2倍周期推导，内部用 Tpwm2 = 2*Tpwm；
 *         2) 老师输出对应PWM2(极性相反)约定，本工程TIM1是
 *            PWM1，输出前按 Tpwm-x 取补。
 *         过调制分支原码 Tx/=T 量纲有误，修正为等比例乘Tpwm2/T。
 */
void SVPWM_Calc(Voltage_Alpha_Beta_DEF v_alpha_beta, float Udc, float Tpwm)
{
    int sector;
    float Tx, Ty, T, Ta, Tb, Tc;
    float Tpwm2;
    float c1, c2, c3;

    if (Udc <= 0.0F || Tpwm <= 0.0F)
    {
        rtY.Tcmp1 = 0.0F;
        rtY.Tcmp2 = 0.0F;
        rtY.Tcmp3 = 0.0F;
        return;
    }

    Tpwm2 = Tpwm * 2.0F;

    /* 矢量扇区选择 (N=4C+2B+A) */
    sector = 0;
    if (v_alpha_beta.Vbeta > 0.0F)
        sector = 1;
    if ((1.73205078F * v_alpha_beta.Valpha - v_alpha_beta.Vbeta) / 2.0F > 0.0F)
        sector += 2;
    if ((-1.73205078F * v_alpha_beta.Valpha - v_alpha_beta.Vbeta) / 2.0F > 0.0F)
        sector += 4;

    /* 计算矢量作用时间 */
    switch (sector)
    {
        case 1:
            Tx = (-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm2 / Udc);
            Ty = (1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm2 / Udc);
            break;

        case 2:
            Tx = (1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm2 / Udc);
            Ty = (-1.73205078F * v_alpha_beta.Vbeta * Tpwm2 / Udc);
            break;

        case 3:
            Tx = -((-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm2 / Udc));
            Ty = 1.73205078F * v_alpha_beta.Vbeta * Tpwm2 / Udc;
            break;

        case 4:
            Tx = -(1.73205078F * v_alpha_beta.Vbeta * Tpwm2 / Udc);
            Ty = (-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm2 / Udc);
            break;

        case 5:
            Tx = 1.73205078F * v_alpha_beta.Vbeta * Tpwm2 / Udc;
            Ty = -((1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm2 / Udc));
            break;

        default:
            Tx = -((1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm2 / Udc));
            Ty = -((-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm2 / Udc));
            break;
    }

    /* 过调制：等比例压缩回线性区 */
    T = Tx + Ty;
    if (T > Tpwm2)
    {
        Tx = Tx * Tpwm2 / T;
        Ty = Ty * Tpwm2 / T;
    }

    /* 扇区切换时间点 */
    Ta = (Tpwm2 - (Tx + Ty)) / 4.0F;
    Tb = Tx / 2.0F + Ta;
    Tc = Ty / 2.0F + Tb;

    switch (sector)
    {
        case 1:
            c1 = Tb; c2 = Ta; c3 = Tc;
            break;

        case 2:
            c1 = Ta; c2 = Tc; c3 = Tb;
            break;

        case 3:
            c1 = Ta; c2 = Tb; c3 = Tc;
            break;

        case 4:
            c1 = Tc; c2 = Tb; c3 = Ta;
            break;

        case 5:
            c1 = Tc; c2 = Ta; c3 = Tb;
            break;

        default:
            c1 = Tb; c2 = Tc; c3 = Ta;
            break;
    }

    /* 老师的分配表是PWM2(极性相反)约定，本工程TIM1为PWM1：CCR取补 */
    rtY.Tcmp1 = Tpwm - c1;
    rtY.Tcmp2 = Tpwm - c2;
    rtY.Tcmp3 = Tpwm - c3;
}

/*
 * 功  能: PARK变换
 * 形  参: alpha_beta电流、COS_SIN值、DQ轴电流
          Id =  Ialpha * cos(theta) + Ibeta * sin(theta);
          Iq = -Ialpha * sin(theta) + Ibeta * cos(theta);
 * 说  明: 交流变直流
 */
void Park_Transf(Current_Alpha_Beta_DEF Current_alpha_beta, Transf_Cos_Sin_DEF cos_sin, Current_DQ_DEF* current_dq)
{
    current_dq->Id =  Current_alpha_beta.Ialpha * cos_sin.Cos + Current_alpha_beta.Ibeta * cos_sin.Sin;
    current_dq->Iq = -Current_alpha_beta.Ialpha * cos_sin.Sin + Current_alpha_beta.Ibeta * cos_sin.Cos;
}

/*
 * 功  能: Clarke变换
 * 形  参: 三相电流以及alpha_beta电流
          Ialpha = ia;
          Ibeta = (ia + 2ib) * sqrt(3)/3;
 * 说  明: 由三相互差120度变换到两相互差90度
 */
void Clarke_Transf(Current_ABC_DEF Current_abc, Current_Alpha_Beta_DEF* Current_alpha_beta)
{
    Current_alpha_beta->Ialpha = Current_abc.Ia;
    Current_alpha_beta->Ibeta = (Current_abc.Ia + Current_abc.Ib * 2.0F) * 0.577350269F;
}

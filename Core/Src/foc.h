#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include <math.h>

#define PI  3.14159265358979f
#define CW  1
#define CCW  -1

typedef struct
{
    float theta;      /* 转子角度 */
    float Udc;        /* 电机供电电压 */
     float Tpwm;       /* 电机PWM周期 */
	 int8_t Dir;
	float speed_ref;
} ExtX;

typedef struct
{
    float Tcmp1;      /* SVPWM输出U项电流 */
    float Tcmp2;      /* SVPWM输出V项电流 */
    float Tcmp3;      /* SVPWM输出W项电流 */
} ExtY;

typedef struct
{
    float Vd;         /* PI控制器输出d轴电压 */
    float Vq;         /* PI控制器输出q轴电压 */
} Voltage_DQ_DEF;

typedef struct
{
    float Cos;        /* 转子角度计算Cos值 */
    float Sin;        /* 转子角度计算Sin值 */
} Transf_Cos_Sin_DEF;

typedef struct
{
    float Valpha;     /* RevPark变换输出Alpha轴电压 */
    float Vbeta;      /* RevPark变换输出Beta轴电压 */
} Voltage_Alpha_Beta_DEF;

typedef struct
{
    float Ia;         /* 电机U项电流 */
    float Ib;         /* 电机V项电流 */
    float Ic;         /* 电机W项电流 */
} Current_ABC_DEF;

typedef struct
{
    float Id;       /* Park变换输出d轴电流 */
    float Iq;       /* Park变换输出q轴电流 */
} Current_DQ_DEF;

typedef struct
{
    float Ialpha;   /* Clarke变换输出Alpha项电流 */
    float Ibeta;    /* Clarke变换输出Beta项电流 */
} Current_Alpha_Beta_DEF;

/* 外部调用变量 */
extern ExtX rtX;
extern ExtY rtY;
extern Transf_Cos_Sin_DEF Transf_Cos_Sin;
extern Voltage_DQ_DEF Voltage_DQ;
extern Voltage_Alpha_Beta_DEF Voltage_Alpha_Beta;
extern Current_ABC_DEF Current_Abc;
extern Current_DQ_DEF Current_dq;
extern Current_Alpha_Beta_DEF Current_Alpbet;
extern Current_DQ_DEF Current_ref;   /* DQ轴电流给定(闭环): 调Current_ref.Iq改Iq指令 */

/* ADC电流采样偏置: 上电零电流时实测, 替代写死的1.25V */
typedef struct
{
    float u;    /* U相(ADC2)零流偏置, 单位V */
    float v;    /* V相(ADC1)零流偏置, 单位V */
    float w;    /* W相(ADC3)零流偏置, 单位V */
} ADC_Offset_DEF;
extern ADC_Offset_DEF ADC_Offset;

/* 外部调用函数 */
extern void ADC_OffsetCalib(void);   /* 上电调用一次, 必须在Motor_Init之后(要CC4触发) */
extern void FOC_Init(void);
extern void FOC_Run(void);
extern void Angle_To_Cos_Sin(float angle, Transf_Cos_Sin_DEF* cos_sin);
extern void Rev_Park_Transf(Voltage_DQ_DEF v_dq, Transf_Cos_Sin_DEF cos_sin, Voltage_Alpha_Beta_DEF* v_alpha_beta);
extern void SVPWM_Calc(Voltage_Alpha_Beta_DEF v_alpha_beta, float Udc, float Tpwm);
void Park_Transf(Current_Alpha_Beta_DEF Current_alpha_beta, Transf_Cos_Sin_DEF cos_sin, Current_DQ_DEF* current_dq);
void Clarke_Transf(Current_ABC_DEF Current_abc, Current_Alpha_Beta_DEF* Current_alpha_beta);
#endif

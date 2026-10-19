#ifndef __PID_H__
#define __PID_H__

#include "main.h"

typedef struct
{
    float Kp;           /* 比例系数 */
    float Ki;           /* 积分系数(已乘采样周期Ts) */
    float Kd;           /* 微分系数(电流环不用,保留字段) */
    float I_sum;        /* 积分累加和 */
    float Up_limit;     /* 输出上限 */
    float Down_limit;   /* 输出下限 */
    float err;          /* 当前误差 */
    float err_Kp;       /* 比例项输出 */
} Current_PID_DEF;

/* D/Q轴电流环各一个PI实例 */
extern Current_PID_DEF Current_D_PID;
extern Current_PID_DEF Current_Q_PID;
extern Current_PID_DEF Speed_PID;
extern Current_PID_DEF Angle_PID;
extern Current_PID_DEF AMO_PID;

void PID_Init(void);
void Current_PID_Calc(float ref, float input, float* output, Current_PID_DEF* current_pid);
void Speed_PID_Calc(float ref, float input, float* output, Current_PID_DEF* Speed_PID);

void position_PID_Calc(float ref, float input, float* output, Current_PID_DEF* position_PID);
void SMO_PID_Calc(float err, float* output, Current_PID_DEF* AMO_PID);
#endif

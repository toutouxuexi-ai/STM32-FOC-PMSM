#include "smo.h"
#include "pid.h"
#include "foc.h"
#include "encoder.h"
SMO_INPUT_DEF SMO_INPUT;
SMO_OUTPUT_DEF SMO_OUTPUT;

float i[2];
float i_pred[2];

float e_lpf[2];

float s[2];
float e[2];
float k = 10.0f;     //最大反电动势等于w*额定转速4000，换算电角速度弧度，1675，0.00517*1675=8.66，可取9以上值
float a = 0.5f;
float k_filtered;

void SMO_Init(void)
{
	
}

void SMO_Run(SMO_INPUT_DEF*SMO_INPUT,SMO_OUTPUT_DEF*SMO_OUTPUT)
{
	
//	k =SMO_OUTPUT->speed_pll * 0.00517f * 1.15f; //≈ 速度 × 0.00595,当前速度E的最大值加15%，自动适应；

//	if(k < 1.0f)  k = 1.0f;   // 最低增益保底
//	if(k > 10.0f) k = 10.0f;  // 最高增益限幅
//	k_filtered = 0.9f * k_filtered + 0.1f * k;
	
	s[0] = i[0] - SMO_INPUT->Ialpha;
	s[1] = i[1] - SMO_INPUT->Ibeta;
	
	 e[0] = k*sat(s[0]);
	 e[1] = k*sat(s[1]);
	
	 e_lpf[0] = a*e_lpf[0] + (1 - a)*e[0];
	 e_lpf[1] = a*e_lpf[1] + (1 - a)*e[1];

    /* 计算k+1时刻观测电流 */
    i_pred[0] = i[0] + Ts_Ls*(SMO_INPUT->Valpha - Rs*i[0] - e[0]);
    i_pred[1] = i[1] + Ts_Ls*(SMO_INPUT->Vbeta - Rs*i[1] - e[1]);

    /* 计算角度、速度 */
    SMO_OUTPUT->theta = -atan2f(e_lpf[0],e_lpf[1]);
    SMO_OUTPUT->e = sqrtf(e_lpf[0]*e_lpf[0] + e_lpf[1]*e_lpf[1]);
    SMO_OUTPUT->speed = SMO_OUTPUT->e/Flux;
	

	SMO_OUTPUT->ed = -e_lpf[0]*cosf(SMO_OUTPUT->theta_pll) - e_lpf[1]*sinf(SMO_OUTPUT->theta_pll);  // ① 鉴相：ê 往估计轴上投影
	
	SMO_OUTPUT->ed /= (SMO_OUTPUT->e + 0.3f);
	
	SMO_PID_Calc(SMO_OUTPUT->ed, &SMO_OUTPUT->speed_pi, &AMO_PID);                       // ② PI：ed → speed_pi
	SMO_OUTPUT->speed_pll = SMO_OUTPUT->speed_pi;        // ③ 关键：相加！
	SMO_OUTPUT->theta_pll += SMO_OUTPUT->speed_pll*Ts;                       // ④ 积分出角度（不变）
	
if(SMO_OUTPUT->theta_pll >= 2*PI) SMO_OUTPUT->theta_pll -= 2*PI;
else if(SMO_OUTPUT->theta_pll < 0.0f)  SMO_OUTPUT->theta_pll += 2*PI;


    // 电角度限幅
    if(SMO_OUTPUT->theta > 2*PI)
        SMO_OUTPUT->theta -= 2*PI;
    if(SMO_OUTPUT->theta < 0)
        SMO_OUTPUT->theta += 2*PI;

    /* 输出估计的反电动势 */
    SMO_OUTPUT->ealpha = e_lpf[0];
    SMO_OUTPUT->ebeta = e_lpf[1];

    i[0] = i_pred[0];
    i[1] = i_pred[1];
}


	
	


static float sat(float x)
{
    if(x > 1.0f)
        return 1.0f;
    if(x < -1.0f)
        return -1.0f;
    return x;
}

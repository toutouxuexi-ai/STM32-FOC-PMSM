#ifndef __pmsm_H__
#define __pmsm_H__
extern float I_uvw[3];
extern float power;
extern float  temper;
void Motor_Run(void);
void Motor_Init(void);

extern float cnt;
#endif 

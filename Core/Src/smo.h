#ifndef __SMO_H__
#define __SMO_H__

#include <math.h>

#define Ts        0.0001f
#define Rs        0.295f
#define Ls        0.0010233f
#define Flux      0.00517f
#define Ts_Ls     0.0977f
#define Ls_Ts     10.233f

typedef struct
{
    float Ialpha;   /* Clarke变换输出Alpha项电流 */
    float Ibeta;    /* Clarke变换输出Beta项电流 */
	float Valpha;
	float Vbeta;
} SMO_INPUT_DEF;

typedef struct
{
    float theta;
	float speed;
	float e;
	float ealpha;
	float ebeta;
	float ed;
	float speed_pll;
	float theta_pll;
	float speed_pi;
} SMO_OUTPUT_DEF;


extern SMO_INPUT_DEF SMO_INPUT;
extern SMO_OUTPUT_DEF SMO_OUTPUT;

extern float k ;

void SMO_Init(void);
static float sat(float x);
#endif

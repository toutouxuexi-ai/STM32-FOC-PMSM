#ifndef __encoder_H__
#define __encoder_H__

#include "main.h"



typedef struct
{
    uint8_t state ;   /* Clarke变换输出Alpha项电流 */
	 uint8_t state_last ;
    float theta[6];    /* Clarke变换输出Beta项电流 */
	float angle;
	float angle_last;    /* 上一有效边沿的角度(统一测速用) */
	uint8_t first_edge;  /* 1=还没来过边沿, 首边沿无基准要丢弃 */
	uint32_t cut;
	float  speed;
	float angle_inc;
	float speed_filter;
	uint16_t edge_timeout;  /* 距上次霍尔边沿的毫秒数(软件超时秒表) */
	uint8_t  stall_cnt;     /* 超时确认计数: 连续多次超时才认故障 */
	uint8_t  stall;         /* 1=停转/堵转(Stall)标志 */
	uint8_t Z;
	uint8_t flag;
} Encoder_Typedef;


extern Encoder_Typedef Encoder;



void Encoder_Init(void);

void SysTick_ISR_encoder(void);
static void Hall_SpeedFilter(void);
void BubbleSort(float* speed, uint8_t length);
void Encoder_ZeroAlignment(void);

static void Encoder_SpeedFilter(void);
#endif

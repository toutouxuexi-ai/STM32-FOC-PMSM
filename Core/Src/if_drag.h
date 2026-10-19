#ifndef __IF_DRAG_H
#define __IF_DRAG_H

#include <stdint.h>

/* 开环拖动的三个阶段 */
typedef enum
{
    IF_IDLE = 0,    /* 空闲: 不输出 */
    IF_RAMPING,     /* 匀加速: 速度从0线性上升到目标 */
    IF_RUNNING      /* 匀速:   保持目标速度 */
} IF_State_t;

/* 启动一次拖动 (内部状态清零, 从静止开始) */
void IF_Drag_Start(void);

/* 停止拖动 */
void IF_Drag_Stop(void);

/* 每个PWM周期调用一次 (100us), 负责更新角度与电流给定 */
void IF_Drag_Update(void);
extern float s_speed_if; 
extern IF_State_t s_state;

#define IF_TRANS_N    200        /* 过渡步数: 200×100us = 20ms */
extern float    trans_offset;   /* 切换时初始相位差(负载角) */
extern uint16_t trans_cnt;      /* 已过渡步数 */
extern float    s_theta_if;     /* 补上开环角度的声明(原来没extern) */

/* 查询当前状态 (外部判断/打印用) */
IF_State_t IF_Drag_GetState(void);

#endif

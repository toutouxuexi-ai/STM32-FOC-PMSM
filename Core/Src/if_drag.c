#include "if_drag.h"
#include "foc.h"          /* 提供 rtX, Current_ref, PI */

/* ===================== 可调参数 (集中放这里) ===================== */
#define IF_TARGET_SPEED    500.0f    /* 目标电角速度 rad/s (约836rpm) */
#define IF_ACCEL           400.0f    /* 匀加速加速度 rad/s^2 */
#define IF_DRAG_IQ         2.0f      /* 固定拖动电流 Iq, 单位 A */
#define IF_TS              0.0001f   /* 本函数调用周期 s (100us) */


float    trans_offset = 0.0f;
uint16_t trans_cnt    = 0;

/* ===================== 内部状态 ===================== */
 IF_State_t s_state = IF_IDLE;
 float s_theta_if;        /* 开环电角度, 范围 0~2pi */
 float s_speed_if;        /* 当前开环电角速度 rad/s */

/* 角度归一化到 0~2pi, 防止一直累加 */
static float Angle_Normalize(float angle)
{
    while (angle >= 2.0f * PI) angle -= 2.0f * PI;
    while (angle < 0.0f)       angle += 2.0f * PI;
    return angle;
}

/* 把固定电流与当前角度输出给FOC */
static void IF_Drag_Output(void)
{
    Current_ref.Id = 0.0f;          /* 表贴电机: id=0, 电流全部给q轴出力 */
    Current_ref.Iq = IF_DRAG_IQ;    /* 固定电流, 不经过速度环 */
    rtX.theta      = s_theta_if;    /* 用开环角度做Park变换 */
}

void IF_Drag_Start(void)
{
    s_state    = IF_RAMPING;
    s_theta_if = 0.0f;
    s_speed_if = 0.0f;
}

void IF_Drag_Stop(void)
{
    s_state    = IF_IDLE;
    s_theta_if = 0.0f;
    s_speed_if = 0.0f;
}

void IF_Drag_Update(void)
{
    switch (s_state)
    {
        case IF_RAMPING:
        {
            /* 速度线性上升 */
            s_speed_if += IF_ACCEL * IF_TS;

            /* 到达目标速度: 钳位并切换到匀速 */
            if (s_speed_if >= IF_TARGET_SPEED)
            {
                s_speed_if = IF_TARGET_SPEED;
                s_state    = IF_RUNNING;
            }

            /* 角度随速度累加 (加速阶段用当前速度) */
            s_theta_if += s_speed_if * IF_TS;
            s_theta_if  = Angle_Normalize(s_theta_if);

            IF_Drag_Output();
            break;
        }

        case IF_RUNNING:
        {
            /* 匀速: 每拍角度增量恒定 = 目标速度 × 周期 */
            s_theta_if += IF_TARGET_SPEED * IF_TS;
            s_theta_if  = Angle_Normalize(s_theta_if);

            IF_Drag_Output();
            break;
        }

        case IF_IDLE:
        default:
            break;
    }
}

IF_State_t IF_Drag_GetState(void)
{
    return s_state;
}

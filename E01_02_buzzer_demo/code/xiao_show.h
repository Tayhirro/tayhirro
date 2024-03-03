/*
 * xiao_show.h
 *
 *  Created on: 2023Äê6ÔÂ21ÈÕ
 *      Author: ×¯²·ÈÙ
 */

#ifndef  CODE_XIAO_SHOW_H_
#define  CODE_XIAO_SHOW_H_

#include "../libraries/zf_common/zf_common_headfile.h"
#include "cpu0_main.h"
#include "xiao_show.h"
#include "xiao_grage.h"
#include "xiao_barrier.h"
#include "xiao_circle.h"

typedef enum{
    Elec,Motor,Pid,Page1,Page2,Page3,Page4,Page5
}SHOW_MODE_enum;
typedef enum{
    SCREEN_IPS200,SCREEN_TFT180
}SHOW_SCREEN_enum;
#define  SHOW_SCREEN   (SCREEN_IPS200)
//#define  SHOW_SCREEN   (SCREEN_TFT180)
void Show_Init(void);
void Show_Switch(SHOW_MODE_enum Show_list[],uint8 Show_max);
void Show_Elec(void);
uint8 Show_Elec_L(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy);
uint8 Show_Elec_Diff(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy);
void Show_Motor(void);
uint8 Show_Motor_Speed(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy);
uint8 Show_Motor_Motor1(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy);
uint8 Show_Motor_Motor2(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy);
void Show_PID(void);
uint8 Show_PID_Elec(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy);
uint8 Show_PID_Motor1(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy);
uint8 Show_PID_Motor2(uint8 x, uint8 y, uint8 zoomx, uint8 zoomy);
void Show_Page1(void);
void Show_Page2(void);
void Show_Page3(void);
void Show_Page4(void);
void Show_Page5(void);
#endif

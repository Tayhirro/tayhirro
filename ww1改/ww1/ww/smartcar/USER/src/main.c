/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"
#include "motor.h"
#include "pid.h"
#include "steer.h"
#include "allinit.h"
#include "pidcc.h" 

/*
 * 系统频率，可查看board.h中的 FOSC 宏定义修改。
 * board.h文件中FOSC的值设置为0,则程序自动设置系统频率为33.1776MHZ
 * 在board_init中,已经将P54引脚设置为复位
 * 如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
 */
extern int adc_left_mid;
extern int adc_right_mid;
extern int adc_mid;
int protect_motor=1;
extern uint16 right_speed_in ;
extern uint16 left_speed_in ;
extern float setpoint_left;
extern float setpoint_right;
void main()
{
	board_init();			// 初始化寄存器,勿删除此句代码。
	
	// 此处编写用户代码(例如：外设初始化代码等)
	ips114_init();			//初始化1.1.4寸ips屏幕
	motor_init();
	steer_init();    
	adc_init(ADC_P01,ADC_SYSclk_DIV_2);
	adc_init(ADC_P00,ADC_SYSclk_DIV_2);
	adc_init(ADC_P05,ADC_SYSclk_DIV_2);
  //ctimer_count_init(CTIM0_P34);
  //ctimer_count_init(CTIM3_P04);
	pit_timer_ms(TIM_0,10);  //中断开启
	//PWMSetSteer(2050);
    while(1)
	{
		
		ips114_clear(WHITE);	
		ips114_showuint16(0,0,adc_left_mid);
		ips114_showuint16(0,2,adc_right_mid);
		ips114_showuint16(0,4,adc_mid);
		//ips114_showuint16(0,0,(uint16)left_speed_in);
		//ips114_showuint16(0,2,(uint16)right_speed_in);
		
	//	ips114_showuint16(0,4,topid_steer->Kd_output_val);
		//ips114_showuint16(0,0,1);
		//ips114_showuint16(0,2,2);
		//delay_ms(1000);
		if(protect_motor){
		PWMSetMotor2(setpoint_left,setpoint_right);}
		else{PWMSetMotor2(0,0);}
		
  }
}



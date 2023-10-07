#include "headfile.h" 
#include "motor.h"
#include "steer.h"
#include "pidcc.h"
void Timer_Init(void){
	ctimer_count_init(CTIM0_P34);
	ctimer_count_init(CTIM3_P04);
	pit_timer_ms(TIM_1,10);  //中断开启
};



void all_init(void)
{ 
	PID_param_init();
	Timer_Init();//中断初始化
	ips114_init();			//初始化1.1.4寸ips屏幕
	motor_init();
	steer_init();    
	adc_init(ADC_P01,ADC_SYSclk_DIV_2);
	adc_init(ADC_P00,ADC_SYSclk_DIV_2);
}
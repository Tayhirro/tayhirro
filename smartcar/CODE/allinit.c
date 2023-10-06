#include "headfile.h" 



void Timer_Init(void){
	ctimer_count_init(CTIM0_P34);
	ctimer_count_init(CTIM3_P04);
	pit_timer_ms(TIM_0,10);  //中断开启
};



void all_init(void)
{ 
	Timer_Init();//中断初始化
}
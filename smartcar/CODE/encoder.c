#include "headfile.h"
#include "encoder.h"
#include "pidcc.h"

float encoder_integral=0;
float encoder_integral1=0,encoder_integral2=0;
float encoder=0;
int  right_speed_in;
int  left_speed_in;
/*******************************************
*************编码器计数函数*****************
*******************************************/
void EncoderCount(void)
{
	   //读取采集到的编码器脉冲数
     right_speed_in = ctimer_count_read(SPEEDR_PLUSE);
		 left_speed_in = ctimer_count_read(SPEEDL_PLUSE);
     //计数器清零
    ctimer_count_clean(SPEEDR_PLUSE);
		ctimer_count_clean(SPEEDL_PLUSE);
		encoder=(right_speed_in+left_speed_in)*0.5;
		encoder_integral=encoder_integral+encoder*0.02;


	 toipid_speed_left->input_val= (int)left_speed_in;      //需要修改
	 toipid_speed_right->input_val= (int)right_speed_in;	
}
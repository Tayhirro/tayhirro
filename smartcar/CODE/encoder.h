#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "headfile.h"

#define SPEEDL_PLUSE   CTIM0_P34
#define SPEEDR_PLUSE   CTIM3_P04

extern int right_speed_in;
extern int left_speed_in;
extern float encoder_integral;
extern float encoder;
void EncoderCount(void);//±àÂëÆ÷¼ÆÊýº¯Êý
#endif
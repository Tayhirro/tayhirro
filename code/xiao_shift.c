/*
 * xiao_shift.c
 *
 *  Created on: 2024年3月13日
 *      Author: Tayhirro
 */
#include "xiao_shift.h"
#include "xiao_image_processing.h"
#include "xiao_trace.h"
#include "xiao_gyroscope.h"
#include "xiao_pid.h"
uint8 none_leftshift_line=0;
uint8 none_rightshift_line=0;
SHIFT_DIRECTION Shift_Direction=SHIFT_DNONE;
int16 Shift_encoderLeft_Thre = 300;              //左轮编码器积分阈值
int16 Shift_encoderRight_Thre = 300;             //右轮编码器积分阈值
int16 Shift_encoderCross_Thre =25000;             //过十字路口的积分阈值
int16 Shift_encoderCross_RUNNING_Thre=90000;
void check_shiftroad(void){
    if (Image_iptsLeftNum < 0.1 / Image_sampleDist) {++none_leftshift_line;}
    if (Image_iptsRightNum < 0.1 / Image_sampleDist){++none_rightshift_line;}
    if(none_leftshift_line>=25){
        //Trace_Status=TRACE_LEFTLOST;
        Shift_Direction=SHIFT_LEFT;
        NORMAL_SPEED=50;
        none_leftshift_line=0;
        //Encoder_Begin(ENCODER_MOTOR_2);
        return;

    }
    if(none_rightshift_line>=25){
        //Trace_Status=TRACE_RIGHTLOST;
        NORMAL_SPEED=50;
        Shift_Direction=SHIFT_RIGHT;
        none_rightshift_line=0;
        //Encoder_Begin(ENCODER_MOTOR_1);
        return;
        //pid_type==PID_INV
    }
    else{
        NORMAL_SPEED=50;
        Shift_Direction=SHIFT_DNONE;
        none_leftshift_line=0;
        none_rightshift_line=0;
    }
}




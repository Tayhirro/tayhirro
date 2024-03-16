/*
 * xiao_shift.c
 *
 *  Created on: 2024年3月13日
 *      Author: Tayhirro
 */
#include "xiao_shift.h"
uint8 none_leftshift_line=0;
uint8 none_rightshift_line=0;
SHIFT_STATUS Shift_Status=SHIFT_NONE;
SHIFT_DIRECTION Shift_Direction=SHIFT_DNONE;
int16 Shift_encoderLeft_Thre = 400;              //左轮编码器积分阈值
int16 Shift_encoderRight_Thre = 400;             //右轮编码器积分阈值
void check_shiftroad(void){
    if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) {++none_leftshift_line;}
    if(none_leftshift_line>=6&&Shift_Status==SHIFT_NONE){
        Trace_Status=TRACE_LEFTLOST;
        Shift_Direction=SHIFT_LEFT;
        none_leftshift_line=0;
        //pid_type==PID_INV
        Shift_Status=SHIFT_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_2);

    }
    if (Image_rptsRightsNum < 0.2 / Image_sampleDist){++none_rightshift_line;}
    if(none_rightshift_line>=6&&Shift_Status==SHIFT_NONE){
        Trace_Status=TRACE_RIGHTLOST;
        Shift_Direction=SHIFT_RIGHT;
        none_rightshift_line=0;
        Shift_Status=SHIFT_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_1);

        //pid_type==PID_INV
    }


}
void handle_shiftroad_left(void){
    if(Shift_Status==SHIFT_BEGIN){
        //if(pid_status==PID_ORIGIN){
        //pwm 打角
        //}
        //
           Shift_Status=SHIFT_RUNNING;
    }
    if(Shift_Status==SHIFT_RUNNING){
        //if(pid_status==PID_ORIGIN){
                //pwm 打角
                //}
            if(abs(Encoder_sum_Motor2)>Shift_encoderLeft_Thre){
                Shift_Status==SHIFT_END;
                Encoder_End(ENCODER_MOTOR_2);
              //  Encoder_Clear(ENCODER_MOTOR_2);
            }
    }
    if(Shift_Status==SHIFT_END){
           Shift_Status=SHIFT_NONE;
           Trace_Status=TRACE_CENTERLINENEAR;

    }
}
void handle_shiftroad_right(void){
    if(Shift_Status==SHIFT_BEGIN){
        //if(pid_status==PID_ORIGIN){
                //pwm 打角
                //}

           Shift_Status=SHIFT_RUNNING;
    }
    if(Shift_Status==SHIFT_RUNNING){
        //if(pid_status==PID_ORIGIN){
                //pwm 打角
                //}
            if(abs(Encoder_sum_Motor1)>Shift_encoderRight_Thre){
                Shift_Status==SHIFT_END;
                Encoder_End(ENCODER_MOTOR_1);
                Encoder_Clear(ENCODER_MOTOR_1);
            }
    }
    if(Shift_Status==SHIFT_END){
           Shift_Status=SHIFT_NONE;
           Trace_Status=TRACE_CENTERLINENEAR;

    }
}



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
int16 Shift_encoderLeft_Thre = 300;              //左轮编码器积分阈值
int16 Shift_encoderRight_Thre = 300;             //右轮编码器积分阈值
int16 Shift_encoderCross_Thre =2000;             //过十字路口的积分阈值
void check_shiftroad(void){
    if (Image_iptsLeftNum < 0.1 / Image_sampleDist) {++none_leftshift_line;}
    if (Image_iptsRightNum < 0.1 / Image_sampleDist){++none_rightshift_line;}
    if(none_rightshift_line>=25&&none_leftshift_line>=10&&Shift_Status==SHIFT_NONE){
        //Trace_Status=TRACE_CROSS;
        Trace_Status=TRACE_CROSS;
        Shift_Direction=SHIFT_CROSS;
        Shift_Status=SHIFT_BEGIN;
        PWMSetSteer(90.0);
        Encoder_Begin(ENCODER_MOTOR_2);
        Encoder_Begin(ENCODER_MOTOR_1);
        none_rightshift_line=0;
        none_leftshift_line=0;
        return;
    }//十字路口
    if(none_leftshift_line>=25&&Shift_Status==SHIFT_NONE){
        Trace_Status=TRACE_LEFTLOST;
        Shift_Direction=SHIFT_LEFT;
        none_leftshift_line=0;
        //pid_type==PID_INV
        Shift_Status=SHIFT_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_2);
        return;

    }
    if(none_rightshift_line>=25&&Shift_Status==SHIFT_NONE){
        Trace_Status=TRACE_RIGHTLOST;
        Shift_Direction=SHIFT_RIGHT;
        none_rightshift_line=0;
        Shift_Status=SHIFT_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_1);
        return;
        //pid_type==PID_INV
    }


}
void handle_shiftroad_cross(void){
    if(Shift_Status==SHIFT_BEGIN){
        if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))/2>Shift_encoderCross_Thre){
                       Shift_Status=SHIFT_END;
                       Encoder_End(ENCODER_MOTOR_2);
                       Encoder_Clear(ENCODER_MOTOR_2);
                       Encoder_End(ENCODER_MOTOR_1);
                       Encoder_Clear(ENCODER_MOTOR_1);
                   }
    }
    if(Shift_Status==SHIFT_END){
            Shift_Status=SHIFT_NONE;
            Trace_Status=TRACE_CENTERLINENEAR;
            Shift_Direction=SHIFT_DNONE;
     }


}
void handle_shiftroad_left(void){
    if(Shift_Status==SHIFT_BEGIN){
        //if(pid_status==PID_ORIGIN){
        //pwm 打角
        //}
        //
        PWMSetSteer(110.0);
           Shift_Status=SHIFT_RUNNING;
    }
    if(Shift_Status==SHIFT_RUNNING){
        //if(pid_status==PID_ORIGIN){
                //pwm 打角
                //}
            if(abs(Encoder_sum_Motor2)>Shift_encoderLeft_Thre){
                Shift_Status=SHIFT_END;
                Encoder_End(ENCODER_MOTOR_2);
                Encoder_Clear(ENCODER_MOTOR_2);
            }
    }
    if(Shift_Status==SHIFT_END){
           Shift_Status=SHIFT_NONE;
           Trace_Status=TRACE_CENTERLINENEAR;
           Shift_Direction=SHIFT_DNONE;
           PWMSetSteer(90);

    }
}
void handle_shiftroad_right(void){
    if(Shift_Status==SHIFT_BEGIN){
        //if(pid_status==PID_ORIGIN){
                //pwm 打角
                //}

        PWMSetSteer(70);
           Shift_Status=SHIFT_RUNNING;
    }
    if(Shift_Status==SHIFT_RUNNING){
        //if(pid_status==PID_ORIGIN){
                //pwm 打角
                //}
            if(abs(Encoder_sum_Motor1)>Shift_encoderRight_Thre){
                Shift_Status=SHIFT_END;
                Encoder_End(ENCODER_MOTOR_1);
                Encoder_Clear(ENCODER_MOTOR_1);
            }
    }
    if(Shift_Status==SHIFT_END){
           Shift_Status=SHIFT_NONE;
           Trace_Status=TRACE_CENTERLINENEAR;
           Shift_Direction=SHIFT_DNONE;
           PWMSetSteer(90);

    }
}



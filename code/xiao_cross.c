/*
 * xiao_cross.c
 *
 *  Created on: 2024年3月3日
 *      Author: Tayhirro
 */
#include "xiao_cross.h"
#include "xiao_encoder.h"
#include "xiao_image_processing.h"
#include "xiao_trace.h"
#include "xiao_gyroscope.h"
#include "xiao_shift.h"
#include "xiao_pid.h"
//----------------------------------------
//----------------------------------------



CROSS_STATUS Cross_status=CROSS_NONE;
//----------------------------------------
uint8 Cross_SteerVar    =   -13;    //打角值
uint8 Cross_Targetangle;            //目标角度值
uint8 Cross_Steer_Angle;        //外部
//------------------------------入十字部分------------------------------
//摄像头 - 环岛大于阈值次数计数 (sc - satisfy condition)
uint8 Cross_camera_scCnt = 0;                  //十字判断摄像头部分满足次数
const uint8 Cross_camera_scCnt_Thre = 0;       //十字判断摄像头满足次数阈值(次数大于阈值就判断预入环)
uint8 Cross_camera_nscCnt = 0;                 //十字判断摄像头部分不满足次数
const uint8 Cross_camera_nscCnt_Thre = 0;      //环岛判断摄像头部分不满足次数阈值(次数大于阈值就判断非环岛)
//----------------------------------------
float Cross_angleEntry_Thre = 180.0;            //入十字角度积分阈值
int16 EncoderCross_In_Thre=2000;
int16 EncoderCross_Thre=10000;
int32 EncoderCross_RUNNING_Thre=64000;
int16 EncoderCross_IN2_Thre=5000;
//陀螺仪测量角度的时候使用的变量 - (假定先使用x轴陀螺仪)
GYROSCOPE_MEASURE_TYPE Cross_measureType = GYROSCOPE_GYRO_Z;

/*
 * @brief               通过检测角点的值,判断是否为十字路口
 * @attention           使用摄像头检测,需要对摄像头进行初始化
 */

void Cross_CheckCamera(void) {
    //十字
    if (Trace_Status==TRACE_CENTERLINENEAR&& Image_LptLeft_Found&&Cross_status == CROSS_NONE) {//Cross_status == CROSS_NONE
        //测试代码
        Trace_Status=TRACE_CROSS;
        Cross_status = CROSS_BEGIN;
        Encoder_End(ENCODER_MOTOR_2);
        Encoder_Clear(ENCODER_MOTOR_2);
        Encoder_End(ENCODER_MOTOR_1);
        Encoder_Clear(ENCODER_MOTOR_1);
        Encoder_Begin(ENCODER_MOTOR_1);
        Encoder_Begin(ENCODER_MOTOR_2);
        Gyroscope_Begin(Cross_measureType);     //开启陀螺仪
//        put_int32(70, 1);
        gpio_set_level(P20_8, GPIO_LOW);
        gpio_set_level(P20_9, GPIO_LOW);
    }
}

void handle_cross(){
    if(Cross_status == CROSS_BEGIN){
           PWMSetSteer(85.0);
           if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>EncoderCross_In_Thre){
                          Encoder_End(ENCODER_MOTOR_2);
                          Encoder_Clear(ENCODER_MOTOR_2);
                          Encoder_End(ENCODER_MOTOR_1);
                          Encoder_Clear(ENCODER_MOTOR_1);
                          Encoder_Begin(ENCODER_MOTOR_1);
                          Encoder_Begin(ENCODER_MOTOR_2);
                          Cross_status = CROSS_IN;
                      }
       }
    if(Cross_status == CROSS_IN){
        Trace_traceType=TRACE_Camera_Far;
        if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>EncoderCross_Thre){
                                      Encoder_End(ENCODER_MOTOR_2);
                                      Encoder_Clear(ENCODER_MOTOR_2);
                                      Encoder_End(ENCODER_MOTOR_1);
                                      Encoder_Clear(ENCODER_MOTOR_1);
                                      Encoder_Begin(ENCODER_MOTOR_1);
                                      Encoder_Begin(ENCODER_MOTOR_2);

                                      Cross_status = CROSS_RUNNING;
        }
    }
       if(Cross_status == CROSS_RUNNING){
           Trace_traceType=TRACE_Camera_Near;
           if(Image_rptsLeftNum>=Image_rptsRightNum){
           if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>3.2*EncoderCross_Thre&&Image_LptLeft_Found){
                   Encoder_End(ENCODER_MOTOR_2);
                   Encoder_Clear(ENCODER_MOTOR_2);
                   Encoder_End(ENCODER_MOTOR_1);
                   Encoder_Clear(ENCODER_MOTOR_1);
                   Encoder_Begin(ENCODER_MOTOR_1);
                   Encoder_Begin(ENCODER_MOTOR_2);
                   Cross_status = CROSS_IN2;
                  }
           }
           if(Image_rptsRightNum>=Image_rptsLeftNum){
                      if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>3.2*EncoderCross_Thre&&Image_LptRight_Found){
                              Encoder_End(ENCODER_MOTOR_2);
                              Encoder_Clear(ENCODER_MOTOR_2);
                              Encoder_End(ENCODER_MOTOR_1);
                              Encoder_Clear(ENCODER_MOTOR_1);
                              Encoder_Begin(ENCODER_MOTOR_1);
                              Encoder_Begin(ENCODER_MOTOR_2);
                              Cross_status = CROSS_IN2;
                             }
                      }
          }
       if(Cross_status == CROSS_IN2){
             if(Shift_Direction==SHIFT_RIGHT){
              PWMSetSteer(82.0);
             }
             if(Shift_Direction==SHIFT_LEFT){
              PWMSetSteer(92.0);
                          }
             if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>EncoderCross_IN2_Thre&&Shift_Direction==SHIFT_BOTH){
                     Encoder_End(ENCODER_MOTOR_2);
                     Encoder_Clear(ENCODER_MOTOR_2);
                     Encoder_End(ENCODER_MOTOR_1);
                     Encoder_Clear(ENCODER_MOTOR_1);
                     Encoder_Begin(ENCODER_MOTOR_1);
                     Encoder_Begin(ENCODER_MOTOR_2);
                     Cross_status = CROSS_END;
                    }
            }
       if(Cross_status == CROSS_END){   //循远线
           Trace_traceType=TRACE_Camera_Far;
          // Trace_traceType=TRACE_Camera_Far;
           if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>EncoderCross_Thre&&Image_iptsLeftNum>10&&Image_iptsRightNum>10){           //重新找到左右线
                                 Cross_status = CROSS_NONE;
                                 Trace_traceType=TRACE_Camera_MID;
                                 Trace_Status=TRACE_CENTERLINENEAR;
                                 Encoder_End(ENCODER_MOTOR_2);
                                 Encoder_Clear(ENCODER_MOTOR_2);
                                 Encoder_End(ENCODER_MOTOR_1);
                                 Encoder_Clear(ENCODER_MOTOR_1);
                             }
       }
}


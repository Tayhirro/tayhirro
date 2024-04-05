/*
 * xiao_cross.c
 *
 *  Created on: 2024��3��3��
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
uint8 Cross_SteerVar    =   -13;    //���ֵ
uint8 Cross_Targetangle;            //Ŀ��Ƕ�ֵ
uint8 Cross_Steer_Angle;        //�ⲿ
//------------------------------��ʮ�ֲ���------------------------------
//����ͷ - ����������ֵ�������� (sc - satisfy condition)
uint8 Cross_camera_scCnt = 0;                  //ʮ���ж�����ͷ�����������
const uint8 Cross_camera_scCnt_Thre = 0;       //ʮ���ж�����ͷ���������ֵ(����������ֵ���ж�Ԥ�뻷)
uint8 Cross_camera_nscCnt = 0;                 //ʮ���ж�����ͷ���ֲ��������
const uint8 Cross_camera_nscCnt_Thre = 0;      //�����ж�����ͷ���ֲ����������ֵ(����������ֵ���жϷǻ���)
//----------------------------------------
float Cross_angleEntry_Thre = 180.0;            //��ʮ�ֽǶȻ�����ֵ
int16 EncoderCross_In_Thre=2000;
int16 EncoderCross_Thre=10000;
int32 EncoderCross_RUNNING_Thre=64000;
int16 EncoderCross_IN2_Thre=5000;
//�����ǲ����Ƕȵ�ʱ��ʹ�õı��� - (�ٶ���ʹ��x��������)
GYROSCOPE_MEASURE_TYPE Cross_measureType = GYROSCOPE_GYRO_Z;

/*
 * @brief               ͨ�����ǵ��ֵ,�ж��Ƿ�Ϊʮ��·��
 * @attention           ʹ������ͷ���,��Ҫ������ͷ���г�ʼ��
 */

void Cross_CheckCamera(void) {
    //ʮ��
    if (Trace_Status==TRACE_CENTERLINENEAR&& Image_LptLeft_Found&&Cross_status == CROSS_NONE) {//Cross_status == CROSS_NONE
        //���Դ���
        Trace_Status=TRACE_CROSS;
        Cross_status = CROSS_BEGIN;
        Encoder_End(ENCODER_MOTOR_2);
        Encoder_Clear(ENCODER_MOTOR_2);
        Encoder_End(ENCODER_MOTOR_1);
        Encoder_Clear(ENCODER_MOTOR_1);
        Encoder_Begin(ENCODER_MOTOR_1);
        Encoder_Begin(ENCODER_MOTOR_2);
        Gyroscope_Begin(Cross_measureType);     //����������
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
       if(Cross_status == CROSS_END){   //ѭԶ��
           Trace_traceType=TRACE_Camera_Far;
          // Trace_traceType=TRACE_Camera_Far;
           if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>EncoderCross_Thre&&Image_iptsLeftNum>10&&Image_iptsRightNum>10){           //�����ҵ�������
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


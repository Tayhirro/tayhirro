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
int16 EncoderCross_In_Thre=1500;
int16 EncoderCross_Thre=10000;
int32 EncoderCross_RUNNING_Thre=64000;
int16 EncoderCross_IN2_Thre=5000;
int16 Gyroscope_z_Cross_in_Thre=10.0;
int16 Gyroscope_z_Cross_running_Thre=280.0;
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



//---------------------CrossѲ��---���������ֺ�������
//void Cross_RunCamera2(){
//    if (Cross_status == CROSS_BEGIN) {
//            Trace_traceType = TRACE_Camera_Near;
//    }
//
//}
//void Cross_RunGyscopAndEncoder(){
//    if (Cross_status == CROSS_BEGIN) {
//        Trace_Status=TRACE_CROSS;
//        Trace_traceType = TRACE_Camera_Near;
//
//
//                //������ǵ�Ͻ���ʱ�򣬿���runningģʽ//���߶���ʱ
//               // if(Image_LptLeft_Found&&Image_LptRight_Found){
//                  // Cross_status = CROSS_RUNNING;
//                  // Trace_traceType = TRACE_Camera_Far_Both;
//                //}
//                   if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
//                                if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
//                                    if (none_left_line_cross>2&&none_right_line_cross>2) {
//                                        none_left_line_cross = 0;
//                                        none_right_line_cross = 0;
//                                        Cross_status = CROSS_RUNNING;
//                                        Trace_traceType = TRACE_Camera_Far_Both;
//                                        Gyroscope_Begin(Cross_measureType);     //����������
//                                    }
//            //------------------------------------------------------------------------------------
//            //------------------------------------------------------------------------------------
//                        //ѭ����ʼ�������ƶ�
//            //------------------------------------------------------------------------------------
//            //------------------------------------------------------------------------------------
//            //------------------------------------------------------------------------------------
//               //
//        }
//                //���ǵ�Ͻ�ʱ���л�CROSS_RUNNINGģʽ  //��ʱ����ͬʱ����Զ��������Զ����������//���߱���������//���������ǻ���//����ͬʱ����
//                else if (Cross_status == CROSS_RUNNING) {
//                              //������Ѱ����
//                            if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //�����޸�||
//                                Encoder_End(ENCODER_MOTOR_1);
//                                Encoder_Clear(ENCODER_MOTOR_1);
//                                Trace_traceType=TRACE_Camera_Near;      //ֻѰ����
//                                Cross_status=CROSS_IN;
//                            }
//
//
//
//
//                                //----------------------------------------
//                }
//
//                else if(Cross_status==CROSS_IN){        //����ʮ��·�ڵ��������
//                                                //�����Ƿ�ʽ���ж��Ƿ񵽴����ս׶�
//                                        if (Cross_measureType == GYROSCOPE_GYRO_X) {
//                                                                               if (fabs(Gyro_x) > Cross_angleEntry_Thre) {
//                                                                                   //���뻷��
//                                                                                   Cross_forceAngle_Status = 0;
//                                                                                   Elec_pidStatus = 1;
//                                                                                   Cross_status = CROSS_IN2;
//
//
//                                                                                   Cross_Steer_Angle=Cross_Targetangle+Cross_SteerVar;  //��������//�����
//
//
//                                                                                   Gyroscope_End(Cross_measureType);
//                                                                                   Gyroscope_Clear(Cross_measureType);
//                                                                                   Gyroscope_Begin(Cross_measureType);
//                                                                               }
//                                                                           }
//                                                                           else if (Cross_measureType == GYROSCOPE_GYRO_Y) {
//                                                                               if (fabs(Gyro_y) > Cross_angleEntry_Thre) {
//                                                                                   //���뻷��
//                                                                                   Cross_forceAngle_Status = 0;
//                                                                                   Elec_pidStatus = 1;
//                                                                                   Cross_status = CROSS_IN2;
//                                                                                   Gyroscope_End(Cross_measureType);
//                                                                                   Gyroscope_Clear(Cross_measureType);
//                                                                                   Gyroscope_Begin(Cross_measureType);
//                                                                               }
//                                                                           }
//                                                                           else if (Cross_measureType == GYROSCOPE_GYRO_Z) {
//                                                                               if (fabs(Gyro_z) > Cross_angleEntry_Thre) {
//                                                                                   //���뻷��
//                                                                                   Cross_forceAngle_Status = 0;
//                                                                                   Elec_pidStatus = 1;
//                                                                                   Speed_set = 30;
//                                                                                   Cross_status = CROSS_IN2;
//                                                                                   Gyroscope_End(Cross_measureType);
//                                                                                   Gyroscope_Clear(Cross_measureType);
//                                                                                   Gyroscope_Begin(Cross_measureType);
//                                                                               }
//                                                                           }
//
//                                    }
//                     else if (Cross_status==CROSS_IN2) {
//                                 //��ʼ�������ߣ��������ߵ�������,ͬʱ�����fabs(Gyro_)���ж��Ƿ񵽴�cross_begin_2�׶�
//
//                                if((Image_LptLeft_Found &&!Image_LptRight_Found)||(Image_LptLeft_Found&&Image_LptRight_Found)){          //��ǵ㷢��֮��
//                                                Cross_status=CROSS_BEGIN2;
//                                                Encoder_Begin(ENCODER_MOTOR_2);
//                                           }
//                            }
//                     else if (Cross_status==CROSS_BEGIN2) {
//
//                         if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
//                                                     if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
//                                                         if (none_left_line_cross>2&&none_right_line_cross>2) {
//                                                             none_left_line_cross = 0;
//                                                             none_right_line_cross = 0;
//                                                             Cross_status = CROSS_RUNNING2;
//                                                             Trace_traceType = TRACE_Camera_Far;
//                                                             Gyroscope_Begin(Cross_measureType);     //����������
//                                                         }
//                                             }
//                     else if(Cross_status==CROSS_RUNNING2){
//                         if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //�����޸�||
//                                                     Encoder_End(ENCODER_MOTOR_2);
//                                                     Encoder_Clear(ENCODER_MOTOR_2);
//                                                     Trace_traceType=TRACE_Camera_Near;
//                                                     Cross_status=CROSS_NONE;
//                                                     Trace_Status=TRACE_CENTERLINENEAR;
//                                                 }
//                     }
//
//}
//void Cross_RunCamera(){
//    if (Cross_status == CROSS_BEGIN) {
//            Trace_traceType = TRACE_Camera_Near;
//
//            //������ǵ�Ͻ���ʱ�򣬿���runningģʽ//���߶���ʱ
//           // if(Image_LptLeft_Found&&Image_LptRight_Found){
//              // Cross_status = CROSS_RUNNING;
//              // Trace_traceType = TRACE_Camera_Far_Both;
//            //}
//               if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
//                            if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
//                                if (none_left_line_cross>2&&none_right_line_cross>2) {
//                                    none_left_line_cross = 0;
//                                    none_right_line_cross = 0;
//                                    Cross_status = CROSS_RUNNING;
//                                    Trace_traceType=TRACE_Camera_Near;
//                                    Gyroscope_Begin(Cross_measureType);     //����������
//                                }
//    }
//            //���ǵ�Ͻ�ʱ���л�CROSS_RUNNINGģʽ  //��ʱ����ͬʱ����Զ��������Զ����������//���߱���������//���������ǻ���//����ͬʱ����
//            else if (Cross_status == CROSS_RUNNING) {
//                          //������Ѱ����
//                        if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //�����޸�||
//                            Encoder_End(ENCODER_MOTOR_2);
//                            Encoder_Clear(ENCODER_MOTOR_2);
//                            Trace_traceType=TRACE_Camera_Near;      //ֻѰ����
//                            //��������
//                            Encoder_Begin(ENCODER_MOTOR_2);
//                            Cross_status=CROSS_IN;
//                        }
//
//
//
//
//                            //----------------------------------------
//            }
//
//            else if(Cross_status==CROSS_IN){        //����ʮ��·�ڵ��������
//                                            //�����Ƿ�ʽ���ж��Ƿ񵽴����ս׶�
//                                    if(1){}
//
//
//
//                                    //�����ǻ��ֲ���
//                                    if (Cross_measureType == GYROSCOPE_GYRO_X) {
//                                                                           if (fabs(Gyro_x) > Cross_angleEntry_Thre) {
//                                                                               //����ʮ�ֻ���
//                                                                               Cross_forceAngle_Status = 0;
//                                                                               Elec_pidStatus = 1;
//                                                                               Cross_status = CROSS_IN2;
//                                                                               Gyroscope_Clear(Cross_measureType);
//                                                                               Gyroscope_Begin(Cross_measureType);
//                                                                           }
//                                                                       }
//                                                                       else if (Cross_measureType == GYROSCOPE_GYRO_Y) {
//                                                                           if (fabs(Gyro_y) > Cross_angleEntry_Thre) {
//                                                                               //����ʮ�ֻ���
//                                                                               Cross_forceAngle_Status = 0;
//                                                                               Elec_pidStatus = 1;
//                                                                               Cross_status = CROSS_IN2;
//                                                                               Gyroscope_End(Cross_measureType);
//                                                                               Gyroscope_Clear(Cross_measureType);
//                                                                               Gyroscope_Begin(Cross_measureType);
//                                                                           }
//                                                                       }
//                                                                       else if (Cross_measureType == GYROSCOPE_GYRO_Z) {
//                                                                           if (fabs(Gyro_z) > Cross_angleEntry_Thre) {
//                                                                               //����ʮ�ֻ���
//                                                                               Cross_forceAngle_Status = 0;
//                                                                               Elec_pidStatus = 1;
//                                                                               Speed_set = 30;
//                                                                               Cross_status = CROSS_IN2;
//                                                                               Gyroscope_End(Cross_measureType);
//                                                                               Gyroscope_Clear(Cross_measureType);
//                                                                               Gyroscope_Begin(Cross_measureType);
//                                                                           }
//                                                                       }
//
//                                }
//
//
//
//
//
//
//
//                 else if (Cross_status==CROSS_IN2) {
//                             //��ʼ�������ߣ��������ߵ�������,ͬʱ�����fabs(Gyro_)���ж��Ƿ񵽴�cross_begin_2�׶�
//
//
//
//                            if((Image_LptLeft_Found &&!Image_LptRight_Found)||(Image_LptLeft_Found&&Image_LptRight_Found)){          //��ǵ㷢��֮��
//                                            Cross_status=CROSS_BEGIN2;
//                                            Encoder_Begin(ENCODER_MOTOR_2);
//                                       }
//                        }
//                 else if (Cross_status==CROSS_BEGIN2) {
//
//                     if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
//                                                 if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
//                                                     if (none_left_line_cross>2&&none_right_line_cross>2) {
//                                                         none_left_line_cross = 0;
//                                                         none_right_line_cross = 0;
//                                                         Cross_status = CROSS_RUNNING2;
//                                                         Trace_traceType = TRACE_Camera_Far;
//                                                         Gyroscope_Begin(Cross_measureType);     //����������
//                                                     }
//                                         }
//                 else if(Cross_status==CROSS_RUNNING2){
//                     if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //�����޸�||
//                                                 Encoder_End(ENCODER_MOTOR_1);
//                                                 Encoder_Clear(ENCODER_MOTOR_1);
//                                                 Trace_traceType=TRACE_Camera_Near;
//                                                 Cross_status=CROSS_NONE;
//                                                 Trace_Status=TRACE_CENTERLINENEAR;
//                                             }
//                 }
//}
void handle_cross(){
    if(Cross_status == CROSS_BEGIN){
           PWMSetSteer(85.0);

           if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>EncoderCross_In_Thre&&fabs(Gyro_z)>Gyroscope_z_Cross_in_Thre){
                          Encoder_End(ENCODER_MOTOR_2);
                          Encoder_Clear(ENCODER_MOTOR_2);
                          Encoder_End(ENCODER_MOTOR_1);
                          Encoder_Clear(ENCODER_MOTOR_1);
                          Encoder_Begin(ENCODER_MOTOR_1);
                          Encoder_Begin(ENCODER_MOTOR_2);

                          Gyroscope_End(Cross_measureType);
                          Gyroscope_Clear(Cross_measureType);
                          Gyroscope_Begin(Cross_measureType);
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
           if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>3.2*EncoderCross_Thre&&Image_LptLeft_Found&&fabs(Gyro_z)>Gyroscope_z_Cross_running_Thre){
                   Encoder_End(ENCODER_MOTOR_2);
                   Encoder_Clear(ENCODER_MOTOR_2);
                   Encoder_End(ENCODER_MOTOR_1);
                   Encoder_Clear(ENCODER_MOTOR_1);
                   Encoder_Begin(ENCODER_MOTOR_1);
                   Encoder_Begin(ENCODER_MOTOR_2);

                   Gyroscope_End(Cross_measureType);
                   Gyroscope_Clear(Cross_measureType);
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


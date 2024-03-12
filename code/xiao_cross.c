/*
 * xiao_cross.c
 *
 *  Created on: 2024��3��3��
 *      Author: Tayhirro
 */
#include "xiao_cross.h"


//----------------------------------------
//----------------------------------------



CROSS_STATUS Cross_status=CROSS_NONE;
int16 Cross_encoderLeft_Thre = 15500;              //���ֱ�����������ֵ
int16 Cross_encoderRight_Thre = 15500;             //���ֱ�����������ֵ
uint8 Cross_forceAngle_Status = 0;             //ʮ��·��ǿ�ƴ��״̬��
//----------------------------------------
uint8 Cross_SteerVar    =   -13;    //���ֵ
uint8 Cross_Targetangle;            //Ŀ��Ƕ�ֵ
uint8 Cross_Steer_Angle;        //�ⲿ
//----------------------------------------
//ʮ�ּ��״̬
uint8 Cross_BeginStatus_Camera = 0;    //����ͷ�жϿ�ʼ״̬��
//uint8 Cross_leftBeginStatus_Elec = 0;      //����ж���ʼ״̬��
//uint8 Cross_rightBeginStatus_Elec = 0;     //����ж��ҿ�ʼ״̬��

//----------------------------------------
//����ͷ - �жϱ����Ƿ���ʧ
uint8 none_left_line_cross = 0,                   //�ж�������Ƿ���ʧ
        none_right_line_cross = 0;                //�ж��ұ����Ƿ���ʧ
uint8 have_left_line_cross = 0,                   //�ж��Ƿ���������
        have_right_line_cross = 0;                //�ж��Ƿ�����ұ���
//------------------------------Ԥ��ʮ�ֲ���------------------------------
int16 Cross_encoderLeft = 0;                       //���ֱ���������
int16 Cross_encoderRight = 0;                      //���ֱ���������
//------------------------------��ʮ�ֲ���------------------------------
//------------�ս���ʮ��·��---------------------//
//����ͷ - ����������ֵ�������� (sc - satisfy condition)
uint8 Cross_camera_scCnt = 0;                  //ʮ���ж�����ͷ�����������
const uint8 Cross_camera_scCnt_Thre = 0;       //ʮ���ж�����ͷ���������ֵ(����������ֵ���ж�Ԥ�뻷)
uint8 Cross_camera_nscCnt = 0;                 //ʮ���ж�����ͷ���ֲ��������
const uint8 Cross_camera_nscCnt_Thre = 0;      //�����ж�����ͷ���ֲ����������ֵ(����������ֵ���жϷǻ���)
//----------------------------------------
float Cross_angleEntry_Thre = 45.0;            //��ʮ�ֽǶȻ�����ֵ


//�����ǲ����Ƕȵ�ʱ��ʹ�õı��� - (�ٶ���ʹ��x��������)
GYROSCOPE_MEASURE_TYPE Cross_measureType = GYROSCOPE_GYRO_X;

/*
 * @brief               ͨ�����ǵ��ֵ,�ж��Ƿ�Ϊʮ��·��
 * @attention           ʹ������ͷ���,��Ҫ������ͷ���г�ʼ��
 */

void Cross_CheckCamera(void) {
    //ʮ��
    if (Trace_Status==TRACE_CENTERLINENEAR&&Cross_status == CROSS_NONE && Image_LptLeft_Found && Image_LptRight_Found) {
        //���Դ���
        Cross_status = CROSS_BEGIN;
        Trace_Status = TRACE_CROSS;
        Encoder_Begin(ENCODER_MOTOR_1);
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
void Cross_RunGyscopAndEncoder(){
    if (Cross_status == CROSS_BEGIN) {
        Trace_Status=TRACE_CROSS;
        Trace_traceType = TRACE_Camera_Near;


                //������ǵ�Ͻ���ʱ�򣬿���runningģʽ//���߶���ʱ
               // if(Image_LptLeft_Found&&Image_LptRight_Found){
                  // Cross_status = CROSS_RUNNING;
                  // Trace_traceType = TRACE_Camera_Far_Both;
                //}
                   if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
                                if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
                                    if (none_left_line_cross>2&&none_right_line_cross>2) {
                                        none_left_line_cross = 0;
                                        none_right_line_cross = 0;
                                        Cross_status = CROSS_RUNNING;
                                        Trace_traceType = TRACE_Camera_Far_Both;
                                        Gyroscope_Begin(Cross_measureType);     //����������
                                    }
            //------------------------------------------------------------------------------------
            //------------------------------------------------------------------------------------
                        //ѭ����ʼ�������ƶ�
            //------------------------------------------------------------------------------------
            //------------------------------------------------------------------------------------
            //------------------------------------------------------------------------------------
               //
        }
                //���ǵ�Ͻ�ʱ���л�CROSS_RUNNINGģʽ  //��ʱ����ͬʱ����Զ��������Զ����������//���߱���������//���������ǻ���//����ͬʱ����
                else if (Cross_status == CROSS_RUNNING) {
                              //������Ѱ����
                            if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //�����޸�||
                                Encoder_End(ENCODER_MOTOR_1);
                                Encoder_Clear(ENCODER_MOTOR_1);
                                Trace_traceType=TRACE_Camera_Near;      //ֻѰ����
                                Cross_status=CROSS_IN;
                            }




                                //----------------------------------------
                }

                else if(Cross_status==CROSS_IN){        //����ʮ��·�ڵ��������
                                                //�����Ƿ�ʽ���ж��Ƿ񵽴����ս׶�
                                        if (Cross_measureType == GYROSCOPE_GYRO_X) {
                                                                               if (fabs(Gyro_x) > Cross_angleEntry_Thre) {
                                                                                   //���뻷��
                                                                                   Cross_forceAngle_Status = 0;
                                                                                   Elec_pidStatus = 1;
                                                                                   Cross_status = CROSS_IN2;


                                                                                   Cross_Steer_Angle=Cross_Targetangle+Cross_SteerVar;  //��������//�����


                                                                                   Gyroscope_End(Cross_measureType);
                                                                                   Gyroscope_Clear(Cross_measureType);
                                                                                   Gyroscope_Begin(Cross_measureType);
                                                                               }
                                                                           }
                                                                           else if (Cross_measureType == GYROSCOPE_GYRO_Y) {
                                                                               if (fabs(Gyro_y) > Cross_angleEntry_Thre) {
                                                                                   //���뻷��
                                                                                   Cross_forceAngle_Status = 0;
                                                                                   Elec_pidStatus = 1;
                                                                                   Cross_status = CROSS_IN2;
                                                                                   Gyroscope_End(Cross_measureType);
                                                                                   Gyroscope_Clear(Cross_measureType);
                                                                                   Gyroscope_Begin(Cross_measureType);
                                                                               }
                                                                           }
                                                                           else if (Cross_measureType == GYROSCOPE_GYRO_Z) {
                                                                               if (fabs(Gyro_z) > Cross_angleEntry_Thre) {
                                                                                   //���뻷��
                                                                                   Cross_forceAngle_Status = 0;
                                                                                   Elec_pidStatus = 1;
                                                                                   Speed_set = 30;
                                                                                   Cross_status = CROSS_IN2;
                                                                                   Gyroscope_End(Cross_measureType);
                                                                                   Gyroscope_Clear(Cross_measureType);
                                                                                   Gyroscope_Begin(Cross_measureType);
                                                                               }
                                                                           }

                                    }







                     else if (Cross_status==CROSS_IN2) {
                                 //��ʼ�������ߣ��������ߵ�������,ͬʱ�����fabs(Gyro_)���ж��Ƿ񵽴�cross_begin_2�׶�



                                if((Image_LptLeft_Found &&!Image_LptRight_Found)||(Image_LptLeft_Found&&Image_LptRight_Found)){          //��ǵ㷢��֮��
                                                Cross_status=CROSS_BEGIN2;
                                                Encoder_Begin(ENCODER_MOTOR_2);
                                           }
                            }
                     else if (Cross_status==CROSS_BEGIN2) {

                         if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
                                                     if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
                                                         if (none_left_line_cross>2&&none_right_line_cross>2) {
                                                             none_left_line_cross = 0;
                                                             none_right_line_cross = 0;
                                                             Cross_status = CROSS_RUNNING2;
                                                             Trace_traceType = TRACE_Camera_Far;
                                                             Gyroscope_Begin(Cross_measureType);     //����������
                                                         }
                                             }
                     else if(Cross_status==CROSS_RUNNING2){
                         if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //�����޸�||
                                                     Encoder_End(ENCODER_MOTOR_1);
                                                     Encoder_Clear(ENCODER_MOTOR_1);
                                                     Trace_traceType=TRACE_Camera_Near;
                                                     Cross_status=CROSS_NONE;
                                                     Trace_Status=TRACE_CENTERLINENEAR;
                                                 }
                     }

}
void Cross_RunCamera(){
    if (Cross_status == CROSS_BEGIN) {
            Trace_traceType = TRACE_Camera_Near;

            //������ǵ�Ͻ���ʱ�򣬿���runningģʽ//���߶���ʱ
           // if(Image_LptLeft_Found&&Image_LptRight_Found){
              // Cross_status = CROSS_RUNNING;
              // Trace_traceType = TRACE_Camera_Far_Both;
            //}
               if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
                            if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
                                if (none_left_line_cross>2&&none_right_line_cross>2) {
                                    none_left_line_cross = 0;
                                    none_right_line_cross = 0;
                                    Cross_status = CROSS_RUNNING;
                                    Trace_traceType=TRACE_Camera_Far_Both;
                                    Gyroscope_Begin(Cross_measureType);     //����������
                                }
    }
            //���ǵ�Ͻ�ʱ���л�CROSS_RUNNINGģʽ  //��ʱ����ͬʱ����Զ��������Զ����������//���߱���������//���������ǻ���//����ͬʱ����
            else if (Cross_status == CROSS_RUNNING) {
                          //������Ѱ����
                        if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //�����޸�||
                            Encoder_End(ENCODER_MOTOR_1);
                            Encoder_Clear(ENCODER_MOTOR_1);
                            Trace_traceType=TRACE_Camera_Near;      //ֻѰ����
                            Cross_status=CROSS_IN;
                        }




                            //----------------------------------------
            }

            else if(Cross_status==CROSS_IN){        //����ʮ��·�ڵ��������
                                            //�����Ƿ�ʽ���ж��Ƿ񵽴����ս׶�
                                    if (Cross_measureType == GYROSCOPE_GYRO_X) {
                                                                           if (fabs(Gyro_x) > Cross_angleEntry_Thre) {
                                                                               //���뻷��
                                                                               Cross_forceAngle_Status = 0;
                                                                               Elec_pidStatus = 1;
                                                                               Cross_status = CROSS_IN2;


                                                                               Cross_Steer_Angle=Cross_Targetangle+Cross_SteerVar;  //��������//�����


                                                                               Gyroscope_End(Cross_measureType);
                                                                               Gyroscope_Clear(Cross_measureType);
                                                                               Gyroscope_Begin(Cross_measureType);
                                                                           }
                                                                       }
                                                                       else if (Cross_measureType == GYROSCOPE_GYRO_Y) {
                                                                           if (fabs(Gyro_y) > Cross_angleEntry_Thre) {
                                                                               //���뻷��
                                                                               Cross_forceAngle_Status = 0;
                                                                               Elec_pidStatus = 1;
                                                                               Cross_status = CROSS_IN2;
                                                                               Gyroscope_End(Cross_measureType);
                                                                               Gyroscope_Clear(Cross_measureType);
                                                                               Gyroscope_Begin(Cross_measureType);
                                                                           }
                                                                       }
                                                                       else if (Cross_measureType == GYROSCOPE_GYRO_Z) {
                                                                           if (fabs(Gyro_z) > Cross_angleEntry_Thre) {
                                                                               //���뻷��
                                                                               Cross_forceAngle_Status = 0;
                                                                               Elec_pidStatus = 1;
                                                                               Speed_set = 30;
                                                                               Cross_status = CROSS_IN2;
                                                                               Gyroscope_End(Cross_measureType);
                                                                               Gyroscope_Clear(Cross_measureType);
                                                                               Gyroscope_Begin(Cross_measureType);
                                                                           }
                                                                       }

                                }







                 else if (Cross_status==CROSS_IN2) {
                             //��ʼ�������ߣ��������ߵ�������,ͬʱ�����fabs(Gyro_)���ж��Ƿ񵽴�cross_begin_2�׶�



                            if((Image_LptLeft_Found &&!Image_LptRight_Found)||(Image_LptLeft_Found&&Image_LptRight_Found)){          //��ǵ㷢��֮��
                                            Cross_status=CROSS_BEGIN2;
                                            Encoder_Begin(ENCODER_MOTOR_2);
                                       }
                        }
                 else if (Cross_status==CROSS_BEGIN2) {

                     if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
                                                 if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
                                                     if (none_left_line_cross>2&&none_right_line_cross>2) {
                                                         none_left_line_cross = 0;
                                                         none_right_line_cross = 0;
                                                         Cross_status = CROSS_RUNNING2;
                                                         Trace_traceType = TRACE_Camera_Far;
                                                         Gyroscope_Begin(Cross_measureType);     //����������
                                                     }
                                         }
                 else if(Cross_status==CROSS_RUNNING2){
                     if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //�����޸�||
                                                 Encoder_End(ENCODER_MOTOR_1);
                                                 Encoder_Clear(ENCODER_MOTOR_1);
                                                 Trace_traceType=TRACE_Camera_Near;
                                                 Cross_status=CROSS_NONE;
                                                 Trace_Status=TRACE_CENTERLINENEAR;
                                             }
                 }
}



/*
 * xiao_trace.c
 *
 *  Created on: 2023��7��2��
 *      Author: Jayden_NaN
 */
#include "xiao_trace.h"
#include "xiao_pid.h"
#include "xiao_motor.h"
#include "xiao_pid.h"
#include "xiao_shift.h"
#include "xiao_cross.h"
#include "xiao_encoder.h"
#include "xiao_image_processing.h"
#include "xiao_circle.h"
//------------------------------------------------------------
//��������
TRACE_TYPE Trace_traceType = TRACE_Camera_MID;
//==============================����ͷѰ�����==============================
//------------------------------------------------------------
//���߱�׼ֵ���
uint8 Trace_middleStandard = 94;                //����ͷ�������е����ڵ���(�����Ͼ�����IMAGE_WIDTH / 2 �м����Ҹ���)
//------------------------------------------------------------
//����Ѳ�ߴ���
float Trace_angleError = 0.0;                   //�Ƕ����

float Trace_angleError_bak[counter_number];               //����
float Trace_angleErrorTher = 7.0;               //�Ƕ������ֵ
uint8 Trace_aimLine = 5;//���1600 ǰհ22 ���1800 ǰհ24                      //���������ҵĵ�n������ΪĿ��ǰհ
int counter=0;

float Trace_lineWeight[] = {0.5, 0.3, 0.2};     //��������ʱ�����м����Ȩ��
int flag=0;

TRACE_CIRCLE_TYPE Trace_Circle_Type=TRACE_CIRCLE_CAREMA_GYROSCOPE_ENCODER;      //��ʼ��Ϊ��ͬ
TRACE_CROSS_TYPE Trace_Cross_Type=TRACE_CROSS_CAREMA_GYROSCOPE_ENCODER;         //��ʼ��Ϊ��ͬ
TRACE_STATUS Trace_Status=TRACE_CENTERLINENEAR;
//------------------------------------------------------------

//==============================���Ѱ�����==============================


//==============================��������==============================

/*
 * @brief               �����ٶ�,������ͼ��ȷ��ǰհ
 * @return              ׷��Ŀ���е����ڵ���
 * @attention           ��Ҫ��ȡ��ǰ���ٶ�(Ҳ����˵��Ҫ���ٶȶ�ȡ���ж�����һֱ��)
 */
static void Trace_GetProspect(void) {
    if (Encoder_readFinishStatus == 1) {
        //ͨ���ٶ�,��̬�ı�ǰհ�ĳ���
        if (Trace_angleError > Trace_angleErrorTher) {
            if (Encoder_1Data > 80 && Encoder_2Data > 80)
                Trace_aimLine += 2;
            else if (Encoder_1Data > 70 && Encoder_2Data > 70)
                Trace_aimLine += 1;
            else
                Trace_aimLine += 0;
        }
        else if (Trace_angleError < -Trace_angleErrorTher) {
            if (Encoder_1Data > 80 && Encoder_2Data > 80)
                Trace_aimLine += 2;
            else if (Encoder_1Data > 70 && Encoder_2Data > 70)
                Trace_aimLine += 1;
            else
                Trace_aimLine += 0;
        }
        else
            Trace_aimLine += 0;
    }
}

/*
 * @brief               ͨ�����м�Ȩ�ػ�ȡangleError
 * @attention           ֻ����������ͷ��,��ŵĻ�û������
 */
static float Trace_GetAngelError() {
    //------------------------------
    //��ȡĿ����
    Trace_GetProspect();

    //------------------------------
    //��ȡ���
    if(Trace_traceType==TRACE_Camera_Far){
        Trace_aimLine=15;
    }
    if(Trace_traceType==TRACE_Camera_MID||Trace_traceType==TRACE_Camera_Near_RIGHT||Trace_traceType==TRACE_Camera_Near_LEFT||Trace_traceType==TRACE_Camera_Near){     //Զ��Ѱ��
        Trace_aimLine=25;
    }

    if(pid_type==PID_INV){
        if(Trace_Status == TRACE_CENTERLINENEAR){
                  /*Trace_angleError = Trace_lineWeight[0] * (float)Image_centerLine[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                                  + Trace_lineWeight[1] * (float)Image_centerLine[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                                  + Trace_lineWeight[2] * (float)Image_centerLine[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];*/

            if (Shift_Direction == SHIFT_RIGHT) {
                Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                                + Trace_lineWeight[1] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                                + Trace_lineWeight[2] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];

            }
            else if (Shift_Direction == SHIFT_LEFT) {
                Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsRightc[bf_clip(Trace_aimLine, 0, Image_rptsRightcNum - 1)][0]
                                + Trace_lineWeight[1] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 1, 0, Image_rptsRightcNum - 1)][0]
                                + Trace_lineWeight[2] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 2, 0, Image_rptsRightcNum - 1)][0];

            }
            else{
//                Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsRightc[bf_clip(Trace_aimLine, 0, Image_rptsRightcNum - 1)][0]
//                                               + Trace_lineWeight[1] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 1, 0, Image_rptsRightcNum - 1)][0]
//                                               + Trace_lineWeight[2] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 2, 0, Image_rptsRightcNum - 1)][0];
                Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                                + Trace_lineWeight[1] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                                + Trace_lineWeight[2] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];
            }
            if(Trace_angleError!=0 && flag==0){Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                              if(counter==counter_number){flag=1;}
                              if(Trace_angleError!=0 && flag==1){
                                  for(int i=0;i<counter_number-1;i++){
                                      Trace_angleError_bak[i]=Trace_angleError_bak[i+1];

                                  }
                                  Trace_angleError_bak[counter-1]=Trace_angleError;
                              }
                              //if(Trace_angleError!=0&& flag==1){counter=counter%counter_number;Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                              if(Trace_angleError==0){Trace_angleError=Trace_angleError_bak[0];}
                               return Trace_angleError;
                  return Trace_angleError;
              }
        if (Trace_Status == TRACE_CROSS) {        //���������ߵ�ʱ��
//                         Trace_angleError = Trace_lineWeight[0] * (float)Image_centerLine[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
//                                         + Trace_lineWeight[1] * (float)Image_centerLine[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
//                                         + Trace_lineWeight[2] * (float)Image_centerLine[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];


//                        if (Shift_Direction == SHIFT_RIGHT) {
//                           Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
//                                           + Trace_lineWeight[1] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
//                                           + Trace_lineWeight[2] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];
//
//                       }
//                       else if (Shift_Direction == SHIFT_LEFT) {
//                           Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsRightc[bf_clip(Trace_aimLine, 0, Image_rptsRightcNum - 1)][0]
//                                           + Trace_lineWeight[1] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 1, 0, Image_rptsRightcNum - 1)][0]
//                                           + Trace_lineWeight[2] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 2, 0, Image_rptsRightcNum - 1)][0];
//                       }
            Trace_aimLine=10;
            
            if(Cross_status==CROSS_RUNNING||Cross_status==CROSS_IN||Cross_status==CROSS_END||Cross_status==CROSS_NONE){
                if(Image_rptsLeftcNum!=0){
            Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                            + Trace_lineWeight[1] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                            + Trace_lineWeight[2] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];
                }
                else {
                    Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsRightc[bf_clip(Trace_aimLine, 0, Image_rptsRightcNum - 1)][0]
                                               + Trace_lineWeight[1] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 1, 0, Image_rptsRightcNum - 1)][0]
                                               + Trace_lineWeight[2] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 2, 0, Image_rptsRightcNum - 1)][0];
                }
                }
                if(Trace_angleError!=0 && flag==0){Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                                     if(counter==counter_number){flag=1;}
                                     if(Trace_angleError!=0 && flag==1){
                                         for(int i=0;i<counter_number-1;i++){
                                             Trace_angleError_bak[i]=Trace_angleError_bak[i+1];

                                         }
                                         Trace_angleError_bak[counter-1]=Trace_angleError;
                                     }
                                     //if(Trace_angleError!=0&& flag==1){counter=counter%counter_number;Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                                     if(Trace_angleError==0){Trace_angleError=Trace_angleError_bak[0];}
                                      return Trace_angleError;
                         return Trace_angleError;
                     }
        if(Trace_Status ==TRACE_CIRCLE_LEFT||Trace_Status ==TRACE_CIRCLE_RIGHT){
            if(Circle_status == CIRCLE_LEFT_BEGIN||Circle_status == CIRCLE_LEFT_RUNNING||Circle_status ==CIRCLE_LEFT_OUT||Circle_status ==CIRCLE_LEFT_END||Circle_status ==CIRCLE_RIGHT_IN||Circle_status ==CIRCLE_RIGHT_IN_PRE){
            Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsRightc[bf_clip(Trace_aimLine, 0, Image_rptsRightcNum - 1)][0]
                                      + Trace_lineWeight[1] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 1, 0, Image_rptsRightcNum - 1)][0]
                                      + Trace_lineWeight[2] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 2, 0, Image_rptsRightcNum - 1)][0];
            }
            if(Circle_status == CIRCLE_RIGHT_BEGIN||Circle_status == CIRCLE_RIGHT_RUNNING||Circle_status ==CIRCLE_RIGHT_OUT||Circle_status ==CIRCLE_RIGHT_END||Circle_status ==CIRCLE_LEFT_IN||Circle_status ==CIRCLE_LEFT_IN_PRE){
                        Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                                                  + Trace_lineWeight[1] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                                                  + Trace_lineWeight[2] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];
                        }

            if(Trace_angleError!=0 && flag==0){Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                              if(counter==counter_number){flag=1;}
                              if(Trace_angleError!=0 && flag==1){
                                  for(int i=0;i<counter_number-1;i++){
                                      Trace_angleError_bak[i]=Trace_angleError_bak[i+1];

                                  }
                                  Trace_angleError_bak[counter-1]=Trace_angleError;
                              }
                              //if(Trace_angleError!=0&& flag==1){counter=counter%counter_number;Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                              if(Trace_angleError==0){Trace_angleError=Trace_angleError_bak[0];}
                               return Trace_angleError;
                  return Trace_angleError;
        }


    return 84;
    }
}

/*
 * @brief               PID��ʼ��(�������ݶ���ʼ��Ϊ0)
 * @return              NULL
 */
void Trace_PIDInit() {
   // PID_Init(&Trace_cameraLeftPID);
   // PID_Init(&Trace_cameraRightPID);
}

/*
 * @brief               Ѱ��PID��������
 * @parameter K_p_set   PID��P����
 * @parameter K_d_set   PID��D����
 * @parameter coLimit   �����޷�
 * @parameter boost
 * @parameter traceType Ѱ������
 * @attention           ������ֻ�ṩ������ͷPID�Ĳ�������,�������������Ч
 */
void Trace_PID_Set(float K_p_set, float K_d_set, float coLimit, float boost, TRACE_TYPE traceType) {
    //------------------------------
    //Ѱ����ͷ�����+�ұ����ҵ������ߵ�PID
    if (traceType == TRACE_Camera_MID) {
            PID_SetParameter(&Trace_cameraMidPID, K_p_set, 0, K_d_set, 0, coLimit, boost);
        }
    //Ѱ����ͷ������ҵ������ߵ�PID

    else if (traceType == TRACE_Camera_Near_LEFT) {

        PID_SetParameter(&Trace_cameraLeftPID, K_p_set, 0, K_d_set, 0, coLimit, boost);
    //Ѱ����ͷ�ұ����ҵ������ߵ�PID
        PID_SetParameter(&Trace_cameraRightPID, K_p_set, 0, K_d_set, 0, coLimit, boost);
    }

}


/*
 * @brief               ����Ѱ��
 * @return              ���������ut
 */
float Trace_Run() {
    //----------------------------------------
    //���Ѱ��
    /*if (Trace_traceType == TRACE_Camera_MID) {
        return 0;
    }*/
    //----------------------------------------
    //����ͷ�����Ѱ����
    if (Trace_Status == TRACE_CENTERLINENEAR) {
           //Trace_GetAngelError();
        if(Shift_Direction==SHIFT_RIGHT){
            //���Ƚ���
//            return Trace_cameraLeftPID.output_val;
////            //Trace_GetAngelError();
////
            direction_control(&Trace_cameraLeftPID,Trace_GetAngelError(),91);
////
////
////            //Trace_PID_Set(Trace_cameraLeftPID.Kp_Set, Trace_cameraLeftPID.Kd_Set, Trace_cameraLeftPID.utLimit, 1.0, Trace_traceType);
////            //PID_PostionalPID(&Trace_cameraLeftPID, 0, Trace_angleError);
        }
        else if (Shift_Direction==SHIFT_LEFT) {
            //���Ƚ���
////               //Trace_GetAngelError();
////
             direction_control(&Trace_cameraRightPID,Trace_GetAngelError(),83);
////
////               //Trace_PID_Set(Trace_cameraRightPID.Kp_Set, Trace_cameraRightPID.Kd_Set, Trace_cameraRightPID.utLimit, 1.0, Trace_traceType);
////               //PID_PostionalPID(&Trace_cameraRightPID, 0, Trace_angleError);
//               return Trace_cameraRightPID.output_val;
           }
           //direction_control(&Trace_cameraMidPID,Trace_angleError,94);
        else{
             direction_control(&Trace_cameraMidPID,Trace_GetAngelError(),91);
        }
           //Trace_PID_Set(Trace_cameraLeftPID.Kp_Set, Trace_cameraLeftPID.Kd_Set, Trace_cameraLeftPID.utLimit, 1.0, Trace_traceType);
           //PID_PostionalPID(&Trace_cameraLeftPID, 0, Trace_angleError);
           return Trace_cameraMidPID.output_val;
       }
    if (Trace_Status ==TRACE_CROSS) {
               //Trace_GetAngelError();
               //direction_control(&Trace_cameraMidPID,Trace_angleError,94);


            if(Cross_status==CROSS_RUNNING||Cross_status==CROSS_NONE||Cross_status==CROSS_IN||Cross_status==CROSS_END){
                if(Image_rptsRightNum<8){
                    direction_control(&Trace_cameraLeftPID,Trace_GetAngelError(),91);
                }
                else if(Image_rptsLeftNum<8){
                    direction_control(&Trace_cameraRightPID,Trace_angleError,84);
                }
                else {
                    direction_control(&Trace_cameraMidPID,Trace_GetAngelError(),91);
                }
            }
               //Trace_PID_Set(Trace_cameraLeftPID.Kp_Set, Trace_cameraLeftPID.Kd_Set, Trace_cameraLeftPID.utLimit, 1.0, Trace_traceType);
               //PID_PostionalPID(&Trace_cameraLeftPID, 0, Trace_angleError);
               return Trace_cameraMidPID.output_val;
           }
    if(Trace_Status==TRACE_CIRCLE_LEFT||Trace_Status==TRACE_CIRCLE_RIGHT){
        if(Circle_status==CIRCLE_LEFT_BEGIN||Circle_status==CIRCLE_LEFT_RUNNING||Circle_status==CIRCLE_LEFT_END){
           // if(Circle_status==CIRCLE_LEFT_IN_PRE){direction_control(&Trace_cameraRightPID,Trace_angleError,91);return Trace_cameraMidPID.output_val;}

            if(Image_rptsLeftNum<8){
                                direction_control(&Trace_cameraRightPID,Trace_GetAngelError(),84);
                            }
                            else if(Image_rptsRightNum<8){
                                direction_control(&Trace_cameraLeftPID,Trace_angleError,91);
                            }
                            else {
                                               direction_control(&Trace_cameraMidPID,Trace_GetAngelError(),84);
                                           }

        }
        if(Circle_status==CIRCLE_RIGHT_RUNNING){
            if(Image_rptsRightNum<8){
                                           direction_control(&Trace_cameraLeftPID,Trace_GetAngelError(),94);
                                       }
                                       else if(Image_rptsLeftNum<8){
                                           direction_control(&Trace_cameraRightPID,Trace_angleError,87);
                                       }
                                       else {
                                                          direction_control(&Trace_cameraMidPID,Trace_GetAngelError(),94);
                                                      }
        }
        if(Circle_status==CIRCLE_RIGHT_BEGIN||Circle_status==CIRCLE_RIGHT_END){
                    if(Image_rptsRightNum<8){
                                                   direction_control(&Trace_cameraLeftPID,Trace_GetAngelError(),91);
                                               }
                                               else if(Image_rptsLeftNum<8){
                                                   direction_control(&Trace_cameraRightPID,Trace_angleError,84);
                                               }
                                               else {
                                                                  direction_control(&Trace_cameraMidPID,Trace_GetAngelError(),91);
                                                              }
                }
        if(Circle_status==CIRCLE_NONE){
            direction_control(&Trace_cameraMidPID,Trace_GetAngelError(),84);
        }
        return Trace_cameraMidPID.output_val;
    }
    else
        return -1;
}

//------------------------------------------------------------
//����vofa�����ú���
void Trace_SetPIDP(float setP, TRACE_TYPE traceType) {
    //------------------------------
    //����Ѱ�����ʱ���P����
    if (traceType == TRACE_Camera_Near_LEFT) {
        Trace_cameraLeftPID.Kp = setP;
        Trace_cameraLeftPID.Kp_Set = setP;
    }
    //------------------------------
    //����Ѱ�ұ���ʱ���P����
    else if (traceType == TRACE_Camera_Near_RIGHT) {
        Trace_cameraRightPID.Kp = setP;
        Trace_cameraRightPID.Kp_Set = setP;

    }
    else if (traceType == TRACE_Camera_MID) {

    }
}


void Trace_SetPIDI(float setI, TRACE_TYPE traceType) {
    //------------------------------
    //����Ѱ�����ʱ���I����
    if (traceType == TRACE_Camera_Near_LEFT) {
        Trace_cameraLeftPID.Ki = setI;
        Trace_cameraLeftPID.Ki_Set = setI;
    }
    //------------------------------
    //����Ѱ�ұ���ʱ���I����
    else if (traceType == TRACE_Camera_Near_RIGHT) {
        Trace_cameraRightPID.Ki = setI;
        Trace_cameraRightPID.Ki_Set = setI;
    }
    else if (traceType == TRACE_Camera_MID) {

    }
}
void Trace_SetPIDD(float setD, TRACE_TYPE traceType) {
    //------------------------------
    //����Ѱ�����ʱ���D����
    if (traceType == TRACE_Camera_Near_LEFT) {
        Trace_cameraLeftPID.Kd = setD;
        Trace_cameraLeftPID.Kd_Set = setD;

    }
    //------------------------------
    //����Ѱ�ұ���ʱ���D����
    else if (traceType == TRACE_Camera_Near_RIGHT) {
        Trace_cameraRightPID.Kd = setD;
        Trace_cameraRightPID.Kd_Set = setD;
    }
    else if (traceType == TRACE_Camera_MID) {

    }
}
void Trace_SetPIDSumLimit(float sumLimit, TRACE_TYPE traceType) {
    //------------------------------
    //����Ѱ�����ʱ���sumLimit����
    if (traceType == TRACE_Camera_Near_LEFT) {
        Trace_cameraLeftPID.sumLimit = sumLimit;
    }
    //------------------------------
    //����Ѱ�ұ���ʱ���sumLimit����
    else if (traceType == TRACE_Camera_Near_RIGHT) {
        Trace_cameraRightPID.sumLimit = sumLimit;
    }
    else if (traceType == TRACE_Camera_MID) {

    }
}
void Trace_SetPIDCoLimit(float coLimit, TRACE_TYPE traceType) {
    //------------------------------
    //����Ѱ�����ʱ���CoLimit����
    if (traceType == TRACE_Camera_Near_LEFT) {
        Trace_cameraLeftPID.utLimit = coLimit;
    }
    //------------------------------
    //����Ѱ�ұ���ʱ���CoLimit����
    else if (traceType == TRACE_Camera_Near_RIGHT) {
        Trace_cameraRightPID.utLimit = coLimit;
    }
    else if (traceType == TRACE_Camera_MID) {

    }
}

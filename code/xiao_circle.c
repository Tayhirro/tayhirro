/*
 * xiao_circle.c
 *
 *  Created on: 2023��7��2��
 *      Author: Jayden_NaN
 */

#include "xiao_circle.h"
#include "xiao_encoder.h"
#include "xiao_image_processing.h"
#include "xiao_trace.h"
#include "xiao_gyroscope.h"
#include "xiao_pid.h"
//------------------------------------------------------------
//״̬��
CIRCLE_STATUS Circle_status = CIRCLE_NONE;
//------------------------------------------------------------
//��������
int32 Circle_encoder;                       //����������,��ֹ����һЩ�ظ��������¼�
int32 Circle_currentEncoder = 0;            //��������ǰ�Ļ���

//----------------------------------------
//�������״̬
uint8 Circle_leftBeginStatus_Camera = 0;    //����ͷ�ж��󻷵���ʼ״̬��
uint8 Circle_rightBeginStatus_Camera = 0;   //����ͷ�ж��һ�����ʼ״̬��
uint8 Circle_leftBeginStatus_Elec = 0;      //����ж��󻷵���ʼ״̬��
uint8 Circle_rightBeginStatus_Elec = 0;     //����ж��һ�����ʼ״̬��

//----------------------------------------
//����ͷ - �жϱ����Ƿ���ʧ
uint8 none_left_line = 0,                   //�ж�������Ƿ���ʧ
        none_right_line = 0;                //�ж��ұ����Ƿ���ʧ
uint8 have_left_line = 0,                   //�ж��Ƿ���������
        have_right_line = 0;                //�ж��Ƿ�����ұ���
//----------------------------------------
//��Ż�������
uint8 Circle_ElecID[6] = {5, 2, 6,          //�����Elec_data�����е�ID��
                        3, 4 , 1};
/*
 * ���ʾ��ͼ:
 * |_1_|--------------------|_2_|--------------------|_3_|
 *                            |
 *                            |
 *                            |
 *                            |
 *                            |
 *                            |
 * |_4_|--------------------|_5_|--------------------|_6_|
 *
 * ע��:
 *      1. ��Ϊ����
 *      2. �������ͨ�������ID�Ž���ӳ��
 *      3. left/right ����ָ�����һ���,�������޹�
 */

//------------------------------��⻷������------------------------------




//----------------------------------------
//���  -  ������ֵ����
int16 Circle_middleElecDiff_Thre = 0;               //�м����ŵ�еĲ�ֵ��ֵ
int16 Circle_middleElec_Front_Thre = 0;             //�м����ŵ��ǰ���е���ֵ
int16 Circle_middleElec_Back_Thre = 0;              //�м����ŵ�к����е���ֵ
int16 Circle_leftElec_Front_Thre = 0;               //ǰ����е���ֵ
int16 Circle_rightElec_Front_Thre = 0;              //ǰ�Ҳ��е���ֵ
int16 Circle_leftElec_Back_Thre = 0;                //������е���ֵ
int16 Circle_rightElec_Back_Thre = 0;               //���Ҳ��е���ֵ

//----------------------------------------




//���  - ����������ֵ�������� (sc - satisfy condition)
uint8 Circle_elecLeft_scCnt = 0;                    //�����жϵ�Ų���������� - �󻷵�
const uint8 Circle_elecLeft_scCnt_Thre = 0;         //�����жϵ�����������ֵ(����������ֵ���ж�Ԥ�뻷)
uint8 Circle_elecLeft_nscCnt = 0;                   //�����жϵ�Ų��ֲ��������
const uint8 Circle_elecLeft_nscCnt_Thre = 5;        //�����жϵ�Ų����������ֵ(����С����ֵ���жϷǻ���)
uint8 Circle_elecRight_scCnt = 0;                   //�����жϵ�Ų���������� - �һ���
const uint8 Circle_elecRight_scCnt_Thre = 0;        //�����жϵ�����������ֵ(����������ֵ���ж�Ԥ�뻷)
uint8 Circle_elecRight_nscCnt = 0;                  //�����жϵ�Ų��ֲ��������
const uint8 Circle_elecRight_nscCnt_Thre = 5;       //�����жϵ�Ų����������ֵ(����С����ֵ���жϷǻ���)





//----------------------------------------
//����ͷ - ����������ֵ�������� (sc - satisfy condition)
uint8 Circle_cameraLeft_scCnt = 0;                  //�����ж�����ͷ�����������
const uint8 Circle_cameraLeft_scCnt_Thre = 0;       //�����ж�����ͷ���������ֵ(����������ֵ���ж�Ԥ�뻷)
uint8 Circle_cameraLeft_nscCnt = 0;                 //�����ж�����ͷ���ֲ��������
const uint8 Circle_cameraLeft_nscCnt_Thre = 0;      //�����ж�����ͷ���ֲ����������ֵ(����������ֵ���жϷǻ���)
uint8 Circle_cameraRight_scCnt = 0;                 //�����ж�����ͷ�����������
const uint8 Circle_cameraRight_scCnt_Thre = 0;      //�����ж�����ͷ���������ֵ(����������ֵ���ж�Ԥ�뻷)
uint8 Circle_cameraRight_nscCnt = 0;                //�����ж�����ͷ���ֲ��������
const uint8 Circle_cameraRight_nscCnt_Thre = 0;     //�����ж�����ͷ���ֲ����������ֵ(����������ֵ���жϷǻ���)

//------------------------------Ԥ�뻷����------------------------------
int16 Circle_encoderLeft = 0;                       //���ֱ���������
int16 Circle_encoderRight = 0;                      //���ֱ���������
//------------------------------����------------------------------
//����ͷ�ж�ʱ��,������������ֵ��16000~19000֮��
int16 Circle_encoderLeft_Thre = 15500;              //���ֱ�����������ֵ
int16 Circle_encoderRight_Thre = 15500;             //���ֱ�����������ֵ
int8 Circle_speedVarLeft_Motor1 = 13;               //ǿ�ƴ�ǵ�ʱ��,���1�ı仯��   -   �󻷵�
int8 Circle_speedVarLeft_Motor2 = -13;              //ǿ�ƴ�ǵ�ʱ��,���2�ı仯��   -   �󻷵�
int8 Circle_speedVarRight_Motor1 = -13;             //ǿ�ƴ�ǵ�ʱ��,���1�ı仯��   -   �һ���
int8 Circle_speedVarRight_Motor2 = 13;              //ǿ�ƴ�ǵ�ʱ��,���2�ı仯��   -   �һ���
uint8 Circle_forceAngleLeft_Status = 0;             //�󻷵�ǿ�ƴ��״̬��
uint8 Circle_forceAngleRight_Status = 0;            //�һ���ǿ�ƴ��״̬��
//--------------------�����ж�--------------------
int16 Circle_encoderLeft_Ther_2ndJd = 16000;        //�����������ж���ֵ           -    �󻷵�
int16 Circle_encoderRight_Ther_2ndJd = 16000;       //�����������ж���ֵ           -    �һ���
uint8 Circle_encoderLeft_isCircle_2ndJd = 0;        //�Ƿ��ǻ���״̬λ            -    �󻷵�
uint8 Circle_encoderRight_isCircle_2ndJd = 0;       //�Ƿ��ǻ���״̬λ            -    �һ���
int16 Circle_Elec_Thre_LeftFront = 3000;            //���μ�����ϵ����ֵ
int16 Circle_Elec_Thre_RightFront = 3000;           //���μ�����ϵ����ֵ

uint8 Circle_isCircle_LeftSC = 0;                   //�󻷵�������������
uint8 Circle_isCircle_LeftSC_Thre = 5;              //�󻷵���������������ֵ
uint8 Circle_isCircle_LeftNSC = 0;                  //�󻷵���������������
uint8 Circle_isCircle_LeftNSC_Thre = 5;             //�󻷵�����������������ֵ

uint8 Circle_isCircle_RightSC = 0;                   //�󻷵�������������
uint8 Circle_isCircle_RightSC_Thre = 5;              //�󻷵���������������ֵ
uint8 Circle_isCircle_RightNSC = 0;                  //�󻷵���������������
uint8 Circle_isCircle_RightNSC_Thre = 5;             //�󻷵�����������������ֵ


//------------------------------�뻷����------------------------------
float Circle_angleLeftEntry_Thre = 45.0;             //�󻷵��뻷�ǶȻ�����ֵ
float Circle_angleRightEntry_Thre = 45.0;            //�һ����뻷�ǶȻ�����ֵ


//------------------------------���ڲ���------------------------------

//------------------------------Ԥ��������------------------------------
float Circle_circleAngle_Thre = 280.0;              //�����ǶȻ�����ֵ(��λΪ��)
uint8 Circle_circleAngle_scCnt = 0;                 //�ǶȻ��ִ�����ֵ��������
uint8 Circle_circleAngle_scCnt_Ther = 3;            //�ǶȻ��ִ�����ֵ����������ֵ
uint8 Circle_circleAngle_nscCnt = 0;                //�ǶȻ��ֲ�������������
uint8 Circle_circleAngel_nscCnt_Thre = 3;           //�ǶȻ��ֲ���������������ֵ

//------------------------------��������------------------------------
int8 Circle_speedVarLeftOut_Motor1 = 8;               //ǿ�ƴ�ǵ�ʱ��,���1�ı仯��   -   �󻷵�
int8 Circle_speedVarLeftOut_Motor2 = -8;              //ǿ�ƴ�ǵ�ʱ��,���2�ı仯��   -   �󻷵�
int8 Circle_speedVarRightOut_Motor1 = -8;             //ǿ�ƴ�ǵ�ʱ��,���1�ı仯��   -   �һ���
int8 Circle_speedVarRightOut_Motor2 = 8;              //ǿ�ƴ�ǵ�ʱ��,���2�ı仯��   -   �һ���
float Circle_angleLeftExit_Thre = 65.0;              //�󻷵������ǶȻ�����ֵ
float Circle_angleRightExit_Thre = 65.0;             //�һ��������ǶȻ�����ֵ

//------------------------------����------------------------------
int16 Circle_encoderExitLeft_Thre = 5000;              //���󻷱�����������ֵ
int16 Circle_encoderExitRight_Thre = 5000;             //���һ�������������ֵ

//------------------------------�໷������------------------------------
uint8 Circle_multiCircle_Status = 0;
//------------------------------����������------------------------------
uint8 Circle_speedAcc_Status = 0;
int16 EncoderCircle_In_Thre =2000;
int16 EncoderCircle_Running_Thre =1500;
int16 EncoderCircle_Out_Thre=6400;
int16 EncoderCircle_End_Thre=4000;
int16 EncoderCircle_Pre_Thre=4000;
//�����ǲ����Ƕȵ�ʱ��ʹ�õı��� - (�ٶ���ʹ��x��������)
GYROSCOPE_MEASURE_TYPE Circle_measureType = GYROSCOPE_GYRO_Z;
float Gyroscope_z_Circle_running_Thre=360.0;

void Circle_CheckCamera(void) {
    //�󻷵�
    if (Circle_status == CIRCLE_NONE && Image_LptLeft_Found && !Image_LptRight_Found && Image_isStraightRight&&Trace_Status==TRACE_CENTERLINENEAR) {
        //���Դ���
        Trace_Status=TRACE_CIRCLE_LEFT;
        Circle_status = CIRCLE_LEFT_BEGIN;
        Trace_traceType = TRACE_Camera_Near_RIGHT;
        Encoder_Begin(ENCODER_MOTOR_2);
        Gyroscope_Begin(Circle_measureType);
//        put_int32(70, 1);
        gpio_set_level(P20_8, GPIO_LOW);
    }
    //�һ���
    if (Circle_status == CIRCLE_NONE && Image_LptRight_Found && !Image_LptLeft_Found && Image_isStraightLeft) {
        Trace_Status=TRACE_CIRCLE_RIGHT;
        Circle_status = CIRCLE_RIGHT_BEGIN;
        Trace_traceType = TRACE_Camera_Near_LEFT;
        Encoder_Begin(ENCODER_MOTOR_1);
//        put_int32(71, 1);
        gpio_set_level(P20_9, GPIO_LOW);
    }
}
void handle_circle_left(){
    //------------------------------�����󻷵�------------------------------
    //------------------------------
    //1. ��ʼ,Ѱ�ұ���
    //2. �뻷,Ѱ��Բ����
    //3. �ڻ���,����Ѳ��(ֱ���õ�� - ����������ͷѰ��Բ����)
    //4. ����,Ѱ��Բ
    //5. �߳�Բ��,Ѱ����(�����õ����)
    //------------------------------
    //��ʼ
    if (Circle_status == CIRCLE_LEFT_BEGIN) {
        Trace_traceType = TRACE_Camera_Near_RIGHT;           //����Ѱ����

        //�ȶ����ߺ�����
        if (Image_rptsLeftNum < 0.2 / Image_sampleDist) {++none_left_line;}
        if (Image_rptsLeftNum > 0.5 / Image_sampleDist && none_left_line > 2) {
            ++have_left_line;
            if (have_left_line >= 1&&abs(Encoder_sum_Motor2)>EncoderCircle_In_Thre) {
                Circle_status = CIRCLE_LEFT_IN_PRE;
                none_left_line = 0;
                have_left_line = 0;
                Encoder_End(ENCODER_MOTOR_2);
                Encoder_Clear(ENCODER_MOTOR_2);
                Encoder_Begin(ENCODER_MOTOR_2);
                Trace_traceType = TRACE_Camera_Near_LEFT;
            }
        }
    }
    else if(Circle_status == CIRCLE_LEFT_IN_PRE){
        PWMSetSteer(98.0);
        if(Image_rptsRightNum==0&&abs(Encoder_sum_Motor2)>EncoderCircle_Pre_Thre){
                        Circle_status=CIRCLE_LEFT_IN;
                        Encoder_End(ENCODER_MOTOR_2);
                        Encoder_Clear(ENCODER_MOTOR_2);
                        Encoder_Begin(ENCODER_MOTOR_2);
             //           Trace_traceType = TRACE_Camera_Near_LEFT;
        }
    }
    //�뻷,Ѱ��Բ����
    else if (Circle_status == CIRCLE_LEFT_IN) {
        //Trace_traceType = TRACE_Camera_Near_LEFT;
        //��⵽�ұ���ʱ�л�����һ��״̬
        PWMSetSteer(100.0);
        if(Image_rptsRightNum!=0&&abs(Encoder_sum_Motor2)*2>EncoderCircle_Running_Thre){
            Circle_status=CIRCLE_LEFT_RUNNING;
            Encoder_End(ENCODER_MOTOR_2);
            Encoder_Clear(ENCODER_MOTOR_2);
            Encoder_Begin(ENCODER_MOTOR_2);
            Trace_traceType = TRACE_Camera_Near_RIGHT;
        }
    }
    //�ڻ���,����Ѳ�� (��������ͷѲ��)
    else if (Circle_status == CIRCLE_LEFT_RUNNING) {
        Trace_traceType = TRACE_Camera_Near_RIGHT;

        //���ҵ���L�ǵ��ʱ��
        if (Image_LptRight_Found) Image_rptsRightsNum = Image_rptsRightcNum = Image_LptRight_rptsRights_id;
        //�⻷�յ�
        if (Image_LptRight_Found && Image_LptRight_rptsRights_id < 0.4 / Image_sampleDist&&abs(Encoder_sum_Motor2)*2>EncoderCircle_Out_Thre){
            Circle_status = CIRCLE_LEFT_OUT;
                        Encoder_End(ENCODER_MOTOR_2);
                        Encoder_Clear(ENCODER_MOTOR_2);
                        Encoder_Begin(ENCODER_MOTOR_2);
                        Trace_traceType = TRACE_Camera_Near_RIGHT;
        }
    }
    //����
    else if (Circle_status == CIRCLE_LEFT_OUT) {
        Trace_traceType = TRACE_Camera_Near_RIGHT;
        PWMSetSteer(100.0);
        if (Image_rptsRightNum>5&&abs(Encoder_sum_Motor2)>EncoderCircle_End_Thre&&fabs(Gyro_z)>Gyroscope_z_Circle_running_Thre) {
            Circle_status = CIRCLE_LEFT_END;
            Encoder_End(ENCODER_MOTOR_2);
            Encoder_Clear(ENCODER_MOTOR_2);
            Encoder_Begin(ENCODER_MOTOR_2);
            Gyroscope_End(Circle_measureType);
            Gyroscope_Clear(Circle_measureType);
            Trace_traceType = TRACE_Camera_Far_RIGHT;
        }
    }
    else if (Circle_status == CIRCLE_LEFT_END) {
        Trace_traceType = TRACE_Camera_Far_RIGHT;
        if(abs(Encoder_sum_Motor2)>1.5*EncoderCircle_End_Thre)
        Trace_traceType=TRACE_Camera_MID;
        Trace_Status=TRACE_CENTERLINENEAR;
        Circle_status = CIRCLE_NONE;
    }

}
void handle_circle_right(){
    //------------------------------�����󻷵�------------------------------
    //------------------------------
    //1. ��ʼ,Ѱ�ұ���
    //2. �뻷,Ѱ��Բ����
    //3. �ڻ���,����Ѳ��(ֱ���õ�� - ����������ͷѰ��Բ����)
    //4. ����,Ѱ��Բ
    //5. �߳�Բ��,Ѱ����(�����õ����)
    //------------------------------
    //��ʼ
    if (Circle_status == CIRCLE_RIGHT_BEGIN) {
        Trace_traceType = TRACE_Camera_Near_LEFT;           //����Ѱ����

        //�ȶ����ߺ�����
        if (Image_rptsRightNum < 0.2 / Image_sampleDist) {++none_right_line;}
        if (Image_rptsRightNum > 0.5 / Image_sampleDist && none_right_line > 2) {
            ++have_right_line;
            if (have_right_line >= 1&&abs(Encoder_sum_Motor1)>EncoderCircle_In_Thre) {
                Circle_status = CIRCLE_RIGHT_IN_PRE;
                none_right_line = 0;
                have_right_line = 0;
                Encoder_End(ENCODER_MOTOR_1);
                Encoder_Clear(ENCODER_MOTOR_1);
                Encoder_Begin(ENCODER_MOTOR_1);
                Trace_traceType = TRACE_Camera_Near_RIGHT;
            }
        }
    }
    else if(Circle_status == CIRCLE_RIGHT_IN_PRE){
        PWMSetSteer(85.0);
        if(Image_rptsLeftNum==0&&abs(Encoder_sum_Motor1)*0.6>EncoderCircle_Pre_Thre){
                        Circle_status=CIRCLE_RIGHT_IN;
                        Encoder_End(ENCODER_MOTOR_1);
                        Encoder_Clear(ENCODER_MOTOR_1);
                        Encoder_Begin(ENCODER_MOTOR_1);
             //           Trace_traceType = TRACE_Camera_Near_LEFT;
        }
    }
    //�뻷,Ѱ��Բ����
    else if (Circle_status == CIRCLE_RIGHT_IN) {
        //Trace_traceType = TRACE_Camera_Near_LEFT;
        //��⵽�ұ���ʱ�л�����һ��״̬
        PWMSetSteer(80.0);
        if(Image_rptsLeftNum!=0&&abs(Encoder_sum_Motor1)*2>EncoderCircle_Running_Thre){
            Circle_status=CIRCLE_RIGHT_RUNNING;
            Encoder_End(ENCODER_MOTOR_1);
            Encoder_Clear(ENCODER_MOTOR_1);
            Encoder_Begin(ENCODER_MOTOR_1);
            Trace_traceType = TRACE_Camera_Near_LEFT;
        }
    }
    //�ڻ���,����Ѳ�� (��������ͷѲ��)
    else if (Circle_status == CIRCLE_RIGHT_RUNNING) {
        Trace_traceType = TRACE_Camera_Near_LEFT;

        //���ҵ���L�ǵ��ʱ��
        if (Image_LptLeft_Found) Image_rptsLeftsNum = Image_rptsLeftcNum = Image_LptLeft_rptsLefts_id;
        //�⻷�յ�
        if (Image_LptLeft_Found && Image_LptLeft_rptsLefts_id < 0.4 / Image_sampleDist&&abs(Encoder_sum_Motor1)*2>EncoderCircle_Out_Thre){
            Circle_status = CIRCLE_RIGHT_OUT;
                        Encoder_End(ENCODER_MOTOR_1);
                        Encoder_Clear(ENCODER_MOTOR_1);
                        Encoder_Begin(ENCODER_MOTOR_1);
                        Trace_traceType = TRACE_Camera_Near_LEFT;
        }
    }
    //����
    else if (Circle_status == CIRCLE_RIGHT_OUT) {
        Trace_traceType = TRACE_Camera_Near_LEFT;
        PWMSetSteer(80.0);
        if (Image_rptsLeftNum>5&&abs(Encoder_sum_Motor1)>EncoderCircle_End_Thre&&fabs(Gyro_z)>Gyroscope_z_Circle_running_Thre) {
            Circle_status = CIRCLE_RIGHT_END;
            Encoder_End(ENCODER_MOTOR_1);
            Encoder_Clear(ENCODER_MOTOR_1);
            Encoder_Begin(ENCODER_MOTOR_1);
            Gyroscope_End(Circle_measureType);
            Gyroscope_Clear(Circle_measureType);
            Trace_traceType = TRACE_Camera_Far_LEFT;
        }
    }
    else if (Circle_status == CIRCLE_RIGHT_END) {
        Trace_traceType = TRACE_Camera_Far_LEFT;
        if(
                abs(Encoder_sum_Motor1)>1.5*EncoderCircle_End_Thre)
        Trace_traceType=TRACE_Camera_MID;
        Trace_Status=TRACE_CENTERLINENEAR;
        Circle_status = CIRCLE_NONE;
    }

}

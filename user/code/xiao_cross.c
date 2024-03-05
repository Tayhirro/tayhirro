/*
 * xiao_cross.c
 *
 *  Created on: 2024��3��3��
 *      Author: Tayhirro
 */
#include "xiao_cross.h"


//----------------------------------------
CROSS_STATUS Cross_status=CROSS_NONE;

//----------------------------------------
//ʮ�ּ��״̬
uint8 Cross_BeginStatus_Camera = 0;    //����ͷ�жϿ�ʼ״̬��
//uint8 Circle_leftBeginStatus_Elec = 0;      //����ж��󻷵���ʼ״̬��
//uint8 Circle_rightBeginStatus_Elec = 0;     //����ж��һ�����ʼ״̬��

//----------------------------------------
//����ͷ - �жϱ����Ƿ���ʧ
uint8 none_left_line_cross = 0,                   //�ж�������Ƿ���ʧ
        none_right_line_cross = 0;                //�ж��ұ����Ƿ���ʧ
uint8 have_left_line_cross = 0,                   //�ж��Ƿ���������
        have_right_line_cross = 0;                //�ж��Ƿ�����ұ���
//----------------------------------------




//------------�ս���ʮ��·��---------------------//
//����ͷ - ����������ֵ�������� (sc - satisfy condition)
uint8 Cross_camera_scCnt = 0;                  //ʮ���ж�����ͷ�����������
const uint8 Cross_camera_scCnt_Thre = 0;       //ʮ���ж�����ͷ���������ֵ(����������ֵ���ж�Ԥ�뻷)
uint8 Cross_camera_nscCnt = 0;                 //ʮ���ж�����ͷ���ֲ��������
const uint8 Cross_camera_nscCnt_Thre = 0;      //�����ж�����ͷ���ֲ����������ֵ(����������ֵ���жϷǻ���)

//------------------------------Ԥ��ʮ�ֲ���------------------------------
int16 Cross_encoderLeft = 0;                       //���ֱ���������
int16 Cross_encoderRight = 0;                      //���ֱ���������
//----------------------------------------
uint8 Cross_forceAngle_Status = 0;             //ʮ��·��ǿ�ƴ��״̬��



//�����ǲ����Ƕȵ�ʱ��ʹ�õı��� - (�ٶ���ʹ��x��������)
GYROSCOPE_MEASURE_TYPE Cross_measureType = GYROSCOPE_GYRO_X;

/*
 * @brief               ͨ�����ǵ��ֵ,�ж��Ƿ�Ϊʮ��·��
 * @attention           ʹ������ͷ���,��Ҫ������ͷ���г�ʼ��
 */

void Cross_CheckCamera(void) {
    //ʮ��
    if (Cross_status == CROSS_NONE && Image_LptLeft_Found && Image_LptRight_Found) {
        //���Դ���
        Cross_status = CROSS_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_2);
//        put_int32(70, 1);
        gpio_set_level(P20_8, GPIO_LOW);
        gpio_set_level(P20_9, GPIO_LOW);
    }
}
void Cross_RunCamera(){
    if (Cross_status == CROSS_BEGIN) {
            Trace_traceType = TRACE_Camera_Near;

            //������ȫ����ʧ
            if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
            if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
                if (none_left_line_cross>2&&none_right_line_cross>2) {
                    Cross_status = CROSS_RUNNING;
                    none_left_line_cross = 0;
                    none_right_line_cross = 0;
                    Gyroscope_Begin(Cross_measureType);
                }
            //���ǵ�Ͻ�ʱ���л�CROSS_RUNNINGģʽ  //��ʱ����ͬʱ����Զ��������Զ����������//���߱���������//����ͬʱ����
            else if (Cross_status == CROSS_RUNNING) {
                        Trace_traceType = TRACE_Camera_Far;
                        //----------------------------------------
                        //�����Ǽ���
                        if (Cross_measureType == GYROSCOPE_GYRO_X) {
                            //�����ݶ�Ϊ>= 90.0,ʵ�ʵĲ��������Ǹ�ֵ,������Ҫ��һ�¸ı�
//                            if (Gyro_x >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
//                                Circle_status = CROSS_RUNNING;
//                                Gyroscope_End(Circle_measureType);
//                            }

                                if(1){           //���������ҵ���Զ��Ҳ�ҵ�
                                    Cross_status=CROSS_OUT;


                                }


                        }
                        else if (Cross_measureType == GYROSCOPE_GYRO_Y) {
//                            if (Gyro_y >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
//                                Circle_status = CROSS_RUNNING;
//                                Gyroscope_End(Circle_measureType);
//                            }
                            if(1){           //���������ҵ���Զ��Ҳ�ҵ�
                                                                Cross_status=CROSS_OUT;


                                                            }
                        }
                        else if (Cross_measureType == GYROSCOPE_GYRO_Z) {
//                            if (Gyro_z >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
//                                Circle_status = CIRCLE_RIGHT_RUNNING;
//                                Gyroscope_End(Circle_measureType);
//                            }
                            if(1){           //���������ҵ���Զ��Ҳ�ҵ�
                                                                                            Cross_status=CROSS_OUT;


                                                                                        }
                        }
                    }
        }


}



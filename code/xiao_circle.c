/*
 * xiao_circle.c
 *
 *  Created on: 2023��7��2��
 *      Author: Jayden_NaN
 */

#include "xiao_circle.h"
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


//�����ǲ����Ƕȵ�ʱ��ʹ�õı��� - (�ٶ���ʹ��x��������)
GYROSCOPE_MEASURE_TYPE Circle_measureType = GYROSCOPE_GYRO_X;

/*
 * @brief               ʹ��ͼ������״̬���
 * @attention           ��Ҫͼ�����г�ʼ��,�����޷�ʹ��
 */
void Circle_PutStatus(void) {
    put_int32(72, Circle_status);
}

/*
 * @brief               ͨ�����ǵ��ֵ,�ж����󻷵������һ���
 * @attention           ʹ������ͷ���,��Ҫ������ͷ���г�ʼ��
 */
void Circle_CheckCamera(void) {
    //�󻷵�
    if (Circle_status == CIRCLE_NONE && Image_LptLeft_Found && !Image_LptRight_Found && Image_isStraightRight) {
        //���Դ���
        Circle_status = CIRCLE_LEFT_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_2);
//        put_int32(70, 1);
        gpio_set_level(P20_8, GPIO_LOW);
    }
    //�һ���
    if (Circle_status == CIRCLE_NONE && Image_LptRight_Found && !Image_LptLeft_Found && Image_isStraightLeft) {
        Circle_status = CIRCLE_RIGHT_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_1);
//        put_int32(71, 1);
        gpio_set_level(P20_9, GPIO_LOW);
    }
}

/*
 * @brief               ��ż���Ƿ���뻷��
 * @attention           Ŀǰʹ�����Ե������к��м�һ����н��л������ж�
 *                      ʹ��������н��д���
 */
static void Circle_CheckElec() {
    //------------------------------
    //�󻷵�����
//    if (Elec_data[Circle_ElecID[1]] > Circle_middleElec_Front_Thre &&
//            (Elec_data[Circle_ElecID[1]] - Elec_data[Circle_ElecID[3]]) > Circle_middleElecDiff_Thre &&
//            Elec_data[Circle_ElecID[0]] > Circle_leftElec_Front_Thre &&
//            Circle_leftBeginStatus_Elec == 0 &&
//            Circle_status == CIRCLE_NONE) {
//        Circle_elecLeft_nscCnt = 0;
//        ++Circle_elecLeft_scCnt;
//        if (Circle_elecLeft_scCnt > Circle_elecLeft_scCnt_Thre)
//            Circle_leftBeginStatus_Elec = 1;
//    }
//    else {
//        if (Circle_elecLeft_scCnt != 0) {
//            ++Circle_elecLeft_nscCnt;
//            if (Circle_elecLeft_nscCnt > Circle_elecLeft_nscCnt_Thre) {
//                Circle_elecLeft_scCnt = 0;
//                Circle_elecLeft_nscCnt = 0;
//            }
//        }
//    }
    //�һ�������
//    if (Elec_data[Circle_ElecID[1]] > Circle_middleElec_Front_Thre &&
//            (Elec_data[Circle_ElecID[1]] - Elec_data[Circle_ElecID[3]]) > Circle_middleElecDiff_Thre &&
//            Elec_data[Circle_ElecID[2]] > Circle_rightElec_Front_Thre &&
//            Circle_rightBeginStatus_Elec == 0 &&
//            Circle_status == CIRCLE_NONE) {
//        Circle_elecRight_nscCnt = 0;
//        ++Circle_elecRight_scCnt;
//        if (Circle_elecRight_scCnt > Circle_elecRight_scCnt_Thre)
//            Circle_rightBeginStatus_Elec = 1;
//    }
//    else {
//        if (Circle_elecRight_scCnt != 0) {
//            ++Circle_elecRight_nscCnt;
//            if (Circle_elecRight_nscCnt > Circle_elecRight_nscCnt_Thre) {
//                Circle_elecRight_scCnt = 0;
//                Circle_elecRight_nscCnt = 0;
//            }
//        }
//    }
}


static void Circle_Begin(CIRCLE_CHECK_METHOD checkMethod) {
    if (Circle_status == CIRCLE_NONE) {
        //����ʹ�õ���ж�
        if (checkMethod == CIRCLE_CHECK_ELEC) {
            if (Circle_leftBeginStatus_Elec == 1) {
                Circle_status = CIRCLE_LEFT_BEGIN;
                Circle_leftBeginStatus_Elec = 0;
            }
            if (Circle_rightBeginStatus_Elec == 1) {
                Circle_status = CIRCLE_RIGHT_BEGIN;
                Circle_rightBeginStatus_Elec = 0;
            }
        }
        //����ʹ������ͷ�ж�
        else if (checkMethod == CIRCLE_CHECK_CAMERA) {
            if (Circle_leftBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_LEFT_BEGIN;
                Circle_leftBeginStatus_Camera = 0;
            }
            if (Circle_rightBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_RIGHT_BEGIN;
                Circle_rightBeginStatus_Camera = 0;
            }
        }
        //����ͷ�͵��ͬʱʹ��
        else if (checkMethod == CIRCLE_CHECK_BOTH) {
            if (Circle_leftBeginStatus_Elec == 1 && Circle_leftBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_LEFT_BEGIN;
                Circle_leftBeginStatus_Camera = 0;
                Circle_leftBeginStatus_Elec = 0;
            }
            if (Circle_rightBeginStatus_Elec == 1 && Circle_rightBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_RIGHT_BEGIN;
                Circle_rightBeginStatus_Camera = 0;
                Circle_rightBeginStatus_Elec = 0;
            }
        }
    }
}


/*
 * @brief               �ڻ�������Ѱ��(ʹ�õ��)
 * @example
 */
void Circle_RunElec(CIRCLE_PRE_ENTRY_METHOD entryMethod) {
    //------------------------------�����󻷵�------------------------------
    //----------------------------------------
    //1. �ҵ������е��λ��
    //      1. ʹ�ñ���������
    //      2. ʹ�õ���ж�
    //      3. ʹ������ͷ�ж�          - Ԥ�뻷
    //2. ʹ�ò���ǿ�ƴ�Ƕ� + �����ǻ���  - �뻷
    //3. ����Ѳ��                     - �ڻ���
    //4. ʹ�õ���ж���ֵ               - Ԥ����
    //      ����ʹ�ýǶȼ���
    //5. ʹ�ò���ǿ�ƴ�Ƕ� + ��ֵ�ж�    - ����
    if (Circle_status == CIRCLE_LEFT_BEGIN) {
        //------------------------------
        //1.Ԥ�뻷ʹ�ñ��������л���
        if (entryMethod == CIRCLE_ENTRY_ENCODER) {

/*
            //���μ��׶�
            if (abs(Encoder_sum_Motor2) > Circle_encoderLeft_Ther_2ndJd && Circle_encoderLeft_isCircle_2ndJd == 0) {
                if (Elec_data[Circle_ElecID[0]] > Circle_Elec_Thre_LeftFront && Elec_data[Circle_ElecID[2]] > Circle_Elec_Thre_RightFront) {
                    Circle_elecLeft_nscCnt = 0;
                    ++Circle_elecLeft_scCnt;
                    if (Circle_elecLeft_scCnt > Circle_elecLeft_scCnt_Thre) {
                        Circle_elecLeft_scCnt = 0;
                        Circle_encoderLeft_isCircle_2ndJd = 1;
                    }
                }
                else if (Circle_encoderLeft_isCircle_2ndJd == 0 && (Elec_data[Circle_ElecID[0]] < Circle_Elec_Thre_LeftFront || Elec_data[Circle_ElecID[2]] < Circle_Elec_Thre_RightFront) && Circle_elecLeft_scCnt != 0) {
                    ++Circle_elecLeft_nscCnt;
                    if (Circle_elecLeft_nscCnt > Circle_elecLeft_nscCnt_Thre) {
                        Circle_elecLeft_scCnt = 0;
                        Circle_elecLeft_nscCnt = 0;
                        Circle_status = CIRCLE_NONE;
                        gpio_set_level(P20_8, GPIO_HIGH);
                    }
                }
            }
*/
            //----------------------------------------
            //ǿ�ƴ�ǽ׶�
            //��������,����ǿ�ƴ��,���������ֹص�(ͬʱ��0),�ǶȻ��ִ�
            if (abs(Encoder_sum_Motor2) > Circle_encoderLeft_Thre && Circle_forceAngleLeft_Status == 0) {
               // Beep_Tweet();
                Elec_pidStatus = 0;
                Motor_1Target = (int16)Speed_set + Circle_speedVarLeft_Motor1;
                Motor_2Target = (int16)Speed_set + Circle_speedVarLeft_Motor2;
                Encoder_End(ENCODER_MOTOR_2);
                Encoder_Clear(ENCODER_MOTOR_2);
                Gyroscope_Begin(Circle_measureType);
                Circle_forceAngleLeft_Status = 1;
                Circle_status = CIRCLE_LEFT_IN;
            }
        }
        //2.Ԥ�뻷ʹ�õ����ֵ�����ж�
       // else if (entryMethod == CIRCLE_ENTRY_ELEC_THRE) {

       // }
        //3.Ԥ�뻷ʹ������ͷ�����ж�
        else if (entryMethod == CIRCLE_ENTRY_CAMERA) {

        }
    }

    //3. ����Ѳ��                     - �ڻ���
    else if (Circle_status == CIRCLE_LEFT_IN) {
        if (Circle_forceAngleLeft_Status == 1) {

            if (Circle_measureType == GYROSCOPE_GYRO_X) {
                if (fabs(Gyro_x) > Circle_angleLeftEntry_Thre) {
                    //���뻷��
                    Circle_forceAngleLeft_Status = 0;
                    Elec_pidStatus = 1;
                    Circle_status = CIRCLE_LEFT_RUNNING;
                }
            }
            else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
                if (fabs(Gyro_y) > Circle_angleLeftEntry_Thre) {
                    //���뻷��
                    Circle_forceAngleLeft_Status = 0;
                    Elec_pidStatus = 1;
                    Circle_status = CIRCLE_LEFT_RUNNING;
                }
            }
            else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
                if (fabs(Gyro_z) > Circle_angleLeftEntry_Thre) {
                    //���뻷��
                    Circle_forceAngleLeft_Status = 0;
                    Elec_pidStatus = 1;
                    Speed_set = 30;
                    Circle_status = CIRCLE_LEFT_RUNNING;
                }
            }
        }
    }
    else if (Circle_status == CIRCLE_LEFT_RUNNING) {
        //Ԥ�����ж�
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            if (fabs(Gyro_x) > Circle_circleAngle_Thre) {
                //׼��������
                Elec_pidStatus = 0;
//                ++Circle_circleAngle_scCnt;
//                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
//                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_LEFT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
//                }
//                else {
//                    if (Circle_circleAngle_scCnt != 0) {
//                        ++Circle_circleAngle_nscCnt;
//                        //���������˵��������
//                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
//                            Circle_circleAngle_nscCnt = 0;
//                            Circle_circleAngle_scCnt = 0;
//                        }
//                    }
//                }
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
            if (Gyro_y > Circle_angleLeftEntry_Thre) {
                //׼��������
                ++Circle_circleAngle_scCnt;
                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_LEFT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
                }
                else {
                    if (Circle_circleAngle_scCnt != 0) {
                        ++Circle_circleAngle_nscCnt;
                        //���������˵��������
                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
                            Circle_circleAngle_nscCnt = 0;
                            Circle_circleAngle_scCnt = 0;
                        }
                    }
                }
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (Gyro_z > Circle_angleLeftEntry_Thre) {
                //׼��������
                ++Circle_circleAngle_scCnt;
                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_LEFT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
                }
                else {
                    if (Circle_circleAngle_scCnt != 0) {
                        ++Circle_circleAngle_nscCnt;
                        //���������˵��������
                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
                            Circle_circleAngle_nscCnt = 0;
                            Circle_circleAngle_scCnt = 0;
                        }
                    }
                }
            }
        }
    }
    else if (Circle_status == CIRCLE_LEFT_OUT) {
        //Ԥ����״̬
        //���pid�ر�,ǿ�ƴ��
        //����״̬�Ժ�,ǿ����ֱ��,Ȼ���ñ��������л���
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            if (fabs(Gyro_x) > Circle_angleLeftExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_2);
                Circle_status = CIRCLE_LEFT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
            if (fabs(Gyro_y) > Circle_angleLeftExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_2);
                Circle_status = CIRCLE_LEFT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (fabs(Gyro_z) > Circle_angleLeftExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_2);
                Circle_status = CIRCLE_LEFT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
    }
    else if (Circle_status == CIRCLE_LEFT_END) {
        if (Encoder_sum_Motor2 > Circle_encoderExitLeft_Thre) {
            Elec_pidStatus = 1;

            if (Circle_multiCircle_Status == 0) {
                Circle_status = CIRCLE_ONLY_ONE;
                if (Circle_speedAcc_Status == 1) {
                    Speed_set += Speed_accVar;
                }
            }
            else {
                Circle_status = CIRCLE_NONE;
            }

            Encoder_End(ENCODER_MOTOR_2);
        }
    }
    //------------------------------�����һ���------------------------------
    //----------------------------------------
    //1. �ҵ������е��λ��
    //      1. ʹ�ñ���������
    //      2. ʹ�õ���ж�
    //      3. ʹ������ͷ�ж�          - Ԥ�뻷
    //2. ʹ�ò���ǿ�ƴ�Ƕ� + �����ǻ���  - �뻷
    //3. ����Ѳ��                     - �ڻ���
    //4. ʹ�õ���ж���ֵ               - Ԥ����
    //5. ʹ�ò���ǿ�ƴ�Ƕ� + ��ֵ�ж�    - ����
    else if (Circle_status == CIRCLE_RIGHT_BEGIN) {
        //------------------------------
        //1.Ԥ�뻷ʹ�ñ��������л���
        if (entryMethod == CIRCLE_ENTRY_ENCODER) {
            //Ϊ�˷�ֹ���Ӱ����ٶ���,����ֱ�ӽ�Elec_pidStatus����Ϊ0
            //Elec_pidStatus = 0;

            //���μ��׶�
/*
            if (abs(Encoder_sum_Motor1) > Circle_encoderRight_Ther_2ndJd && Circle_encoderRight_isCircle_2ndJd == 0) {
                if (Elec_data[Circle_ElecID[0]] > Circle_Elec_Thre_LeftFront && Elec_data[Circle_ElecID[2]] > Circle_Elec_Thre_RightFront) {
                    Circle_elecRight_nscCnt = 0;
                    ++Circle_elecRight_scCnt;
                    if (Circle_elecRight_scCnt > Circle_elecRight_scCnt_Thre) {
                        Circle_elecRight_scCnt = 0;
                        Circle_encoderRight_isCircle_2ndJd = 1;
                    }
                }
                else if ((Elec_data[Circle_ElecID[0]] < Circle_Elec_Thre_LeftFront || Elec_data[Circle_ElecID[2]] < Circle_Elec_Thre_RightFront) && Circle_elecLeft_scCnt != 0) {
                    ++Circle_elecRight_nscCnt;
                    if (Circle_elecRight_nscCnt > Circle_elecRight_scCnt_Thre) {
                        Circle_elecRight_scCnt = 0;
                        Circle_elecRight_nscCnt = 0;
                        Circle_status = CIRCLE_NONE;
                        gpio_set_level(P20_9, GPIO_HIGH);
                    }
                }
            }
*/

            //----------------------------------------
            //ǿ�ƴ�ǽ׶�
            //��������,����ǿ�ƴ��,���������ֹص�(ͬʱ��0),�ǶȻ��ִ�
            if (abs(Encoder_sum_Motor1) > Circle_encoderRight_Thre && Circle_forceAngleRight_Status == 0) {
                Elec_pidStatus = 0;
                Motor_1Target = (int16)Speed_set + Circle_speedVarRight_Motor1;
                Motor_2Target = (int16)Speed_set + Circle_speedVarRight_Motor2;
                Encoder_End(ENCODER_MOTOR_1);
                Encoder_Clear(ENCODER_MOTOR_1);
                Gyroscope_Begin(Circle_measureType);
                Circle_forceAngleRight_Status = 1;
                Circle_status = CIRCLE_RIGHT_IN;
            }
        }
        //2.Ԥ�뻷ʹ�õ����ֵ�����ж�
        else if (entryMethod == CIRCLE_ENTRY_ELEC_THRE) {

        }
        //3.Ԥ�뻷ʹ������ͷ�����ж�
        else if (entryMethod == CIRCLE_ENTRY_CAMERA) {

        }
    }
    else if (Circle_status == CIRCLE_RIGHT_IN) {
        if (Circle_forceAngleRight_Status == 1) {
            if (Circle_measureType == GYROSCOPE_GYRO_X) {
                if (fabs(Gyro_x) > Circle_angleRightEntry_Thre) {
                    //���뻷��
                    Circle_forceAngleRight_Status = 0;
                    Elec_pidStatus = 1;
                    Circle_status = CIRCLE_RIGHT_RUNNING;
                }
            }
            else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
                if (fabs(Gyro_y) > Circle_angleRightEntry_Thre) {
                    //���뻷��
                    Circle_forceAngleRight_Status = 0;
                    Elec_pidStatus = 1;
                    Circle_status = CIRCLE_RIGHT_RUNNING;
                }
            }
            else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
                if (fabs(Gyro_z) > Circle_angleRightEntry_Thre) {
                    //���뻷��
                    Circle_forceAngleRight_Status = 0;
                    Elec_pidStatus = 1;
                    Circle_status = CIRCLE_RIGHT_RUNNING;
                }
            }
        }
    }
    else if (Circle_status == CIRCLE_RIGHT_RUNNING) {
        //Ԥ�����ж�
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            if (fabs(Gyro_x) > Circle_circleAngle_Thre) {
                //׼��������
//                ++Circle_circleAngle_scCnt;
//                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_RIGHT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarRightOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarRightOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
//                }
//                else {
//                    if (Circle_circleAngle_scCnt != 0) {
//                        ++Circle_circleAngle_nscCnt;
//                        //���������˵��������
//                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
//                            Circle_circleAngle_nscCnt = 0;
//                            Circle_circleAngle_scCnt = 0;
//                        }
//                    }
//                }
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
            if (fabs(Gyro_y) > Circle_angleRightEntry_Thre) {
                //׼��������
                ++Circle_circleAngle_scCnt;
                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_RIGHT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarRightOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarRightOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
                }
                else {
                    if (Circle_circleAngle_scCnt != 0) {
                        ++Circle_circleAngle_nscCnt;
                        //���������˵��������
                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
                            Circle_circleAngle_nscCnt = 0;
                            Circle_circleAngle_scCnt = 0;
                        }
                    }
                }
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (fabs(Gyro_z) > Circle_angleRightEntry_Thre) {
                //׼��������
                ++Circle_circleAngle_scCnt;
                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_RIGHT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarRightOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarRightOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
                }
                else {
                    if (Circle_circleAngle_scCnt != 0) {
                        ++Circle_circleAngle_nscCnt;
                        //���������˵��������
                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
                            Circle_circleAngle_nscCnt = 0;
                            Circle_circleAngle_scCnt = 0;
                        }
                    }
                }
            }
        }
    }
    else if (Circle_status == CIRCLE_RIGHT_OUT) {
        //Ԥ����״̬
        //���pid�ر�,ǿ�ƴ��
        //����״̬�Ժ�,ǿ����ֱ��,Ȼ���ñ��������л���
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            if (fabs(Gyro_x) > Circle_angleRightExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_1);
                Circle_status = CIRCLE_RIGHT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (fabs(Circle_measureType) == GYROSCOPE_GYRO_Y) {
            if (Gyro_y > Circle_angleRightExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_1);
                Circle_status = CIRCLE_RIGHT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (fabs(Gyro_z) > Circle_angleRightExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_1);
                Circle_status = CIRCLE_RIGHT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
    }
    else if (Circle_status == CIRCLE_RIGHT_END) {
        if (Encoder_sum_Motor1 > Circle_encoderExitRight_Thre) {
            Elec_pidStatus = 1;
            Encoder_End(ENCODER_MOTOR_1);
            if (Circle_multiCircle_Status == 0) {
                Circle_status = CIRCLE_ONLY_ONE;
                if (Circle_speedAcc_Status == 1) {
                    Speed_set += Speed_accVar;
                }
            }
            else {
                Circle_status = CIRCLE_NONE;
            }
        }
    }
}




/*
 * @brief               �ڻ�������Ѱ��(ʹ������ͷ)
 * @attention           ʹ�õ���������,�ǵó�ʼ��
 * @example
 */
void Circle_RunCamera() {
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
        Trace_traceType = TRACE_Camera_RIGHT;

        //�ȶ����ߺ�����
        if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) {++none_left_line;Trace_Status=TRACE_LEFTLOST;}
        if (Image_rptsLeftsNum > 1.0 / Image_sampleDist && none_left_line > 2) {
            ++have_left_line;
            if (have_left_line > 1) {
                Circle_status = CIRCLE_LEFT_IN;
                none_left_line = 0;
                have_left_line = 0;
                Gyroscope_Begin(Circle_measureType);
            }
        }
    }
    //�뻷,Ѱ��Բ����
    else if (Circle_status == CIRCLE_LEFT_IN) {
        Trace_traceType = TRACE_Camera_LEFT;
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            //�����ݶ�Ϊ>= 90.0,ʵ�ʵĲ��������Ǹ�ֵ,������Ҫ��һ�¸ı�
            if (Gyro_x >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_LEFT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
            if (Gyro_y >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_LEFT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (Gyro_z >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_LEFT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
    }
    //�ڻ���,����Ѳ�� (��������ͷѲ��)
    else if (Circle_status == CIRCLE_LEFT_RUNNING) {
        Trace_traceType = TRACE_Camera_RIGHT;

        //���ҵ���L�ǵ��ʱ��
        if (Image_LptRight_Found) Image_rptsRightsNum = Image_rptsRightcNum = Image_LptRight_rptsRights_id;
        //�⻷�յ�
        if (Image_LptRight_Found && Image_LptRight_rptsRights_id < 0.4 / Image_sampleDist)
            Circle_status = CIRCLE_LEFT_OUT;
    }
    //����,Ѱ��Բ
    else if (Circle_status == CIRCLE_LEFT_OUT) {
        Trace_traceType = TRACE_Camera_LEFT;

        //����Ϊ��ֱ��
        if (Image_isStraightRight) {
            Circle_status = CIRCLE_LEFT_END;
        }
    }
    //�߳�Բ��,Ѱ����(�õ����)
    else if (Circle_status == CIRCLE_LEFT_END) {
        Trace_traceType = TRACE_Camera_RIGHT;

        //�����ȶ�����
        if (Image_rptsLeftsNum < 0.2 / Image_sampleDist)
            ++none_left_line;
        if (Image_rptsLeftsNum > 1 / Image_sampleDist && none_left_line) {
            Circle_status = CIRCLE_NONE;
            none_left_line = 0;
        }
    }

    //------------------------------�����һ���------------------------------
    //------------------------------
    //1. ��ʼ,Ѱ�����
    //2. �뻷,Ѱ��Բ����
    //3. �ڻ���,����Ѳ��(ֱ���õ�� - ����������ͷѰ��Բ����)
    //4. ����,Ѱ��Բ
    //5. �߳�Բ��,Ѱ����(�����õ����)
    //------------------------------
    //1. ��ʼ,Ѱ�����
    else if (Circle_status == CIRCLE_RIGHT_BEGIN) {
        Trace_traceType = TRACE_Camera_LEFT;
        //�ȶ����ߺ�����
        if (Image_rptsRightsNum < 0.2 / Image_sampleDist){
            ++none_right_line;
            Trace_Status=TRACE_RIGHTLOST;
        }
        if (Image_rptsRightsNum > 1.0 / Image_sampleDist && none_right_line > 2) {
            ++have_right_line;
            if (have_right_line > 1) {
                Circle_status = CIRCLE_RIGHT_IN;
                none_right_line = 0;
                have_right_line = 0;
                Gyroscope_Begin(Circle_measureType);
            }
        }
    }
    //2. �뻷,Ѱ��Բ����
    else if (Circle_status == CIRCLE_RIGHT_IN) {
        Trace_traceType = TRACE_Camera_RIGHT;
        //----------------------------------------
        //�����Ǽ���
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            //�����ݶ�Ϊ>= 90.0,ʵ�ʵĲ��������Ǹ�ֵ,������Ҫ��һ�¸ı�
            if (Gyro_x >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_RIGHT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
            if (Gyro_y >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_RIGHT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (Gyro_z >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_RIGHT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
    }
    //3. �ڻ���,����Ѳ��(ֱ���õ�� - ����������ͷѰ��Բ����)
    else if (Circle_status == CIRCLE_RIGHT_RUNNING) {
        Trace_traceType = TRACE_Camera_LEFT;
        //���ҵ���L�ǵ��ʱ��
        if (Image_LptLeft_Found) Image_rptsLeftsNum = Image_rptsLeftcNum = Image_LptLeft_rptsLefts_id;
        //�⻷�յ�
        if (Image_LptLeft_Found && Image_LptLeft_rptsLefts_id < 0.4 / Image_sampleDist)
            Circle_status = CIRCLE_LEFT_OUT;
    }
    //4. ����,Ѱ��Բ
    else if (Circle_status == CIRCLE_RIGHT_OUT) {
        Trace_traceType = TRACE_Camera_RIGHT;
        if (Image_isStraightLeft)
            Circle_status = CIRCLE_RIGHT_END;
    }
    //5. �߳�Բ��,Ѱ����(�����õ����)
    else if (Circle_status == CIRCLE_RIGHT_END) {
        Trace_traceType = TRACE_Camera_LEFT;
        //�����ȶ�����
        if (Image_rptsRightsNum < 0.2 / Image_sampleDist)
            ++none_right_line;
        if (Image_rptsRightsNum > 0.2 / Image_sampleDist && none_right_line > 2) {
            Circle_status = CIRCLE_NONE;
            none_right_line = 0;
        }
    }
}


/*
 * @brief                   ������
 * @parameter checkMethod   ����뻷�ķ���
 * @parameter runMethod     ����Ѱ���ķ���
 * @attention               1. �������ֽ����󻷵� - ʹ�ñ�����2
 *                             �������ֽ����һ��� - ʹ�ñ�����1
 * @example
 */
void Circle_Handler(CIRCLE_CHECK_METHOD checkMethod, CIRCLE_RUN_METHOD runMethod, CIRCLE_PRE_ENTRY_METHOD entryMethod) {
    //----------------------------------------
    //����Ƿ��뻷
    //��ʹ������ͷ���
    if (Circle_status == CIRCLE_NONE) {
        if (checkMethod == CIRCLE_CHECK_CAMERA) {
            Circle_CheckCamera();
            if (Circle_leftBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_LEFT_BEGIN;
                Circle_leftBeginStatus_Camera = 0;
                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
                    Encoder_Begin(ENCODER_MOTOR_2);
                }
            }
            else if (Circle_rightBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_RIGHT_BEGIN;
                Circle_rightBeginStatus_Camera = 0;
                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
                    Encoder_Begin(ENCODER_MOTOR_1);
                }
            }
        }
        //��ʹ�õ�ż��
//        else if (checkMethod == CIRCLE_CHECK_ELEC) {
//            Circle_CheckElec();
//            if (Circle_leftBeginStatus_Elec == 1) {
//                Circle_status = CIRCLE_LEFT_BEGIN;
//                Circle_leftBeginStatus_Elec = 0;
//                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
//                    Encoder_Begin(ENCODER_MOTOR_2);
//                }
//            }
//            else if (Circle_rightBeginStatus_Elec == 1) {
//                Circle_status = CIRCLE_RIGHT_BEGIN;
//                Circle_rightBeginStatus_Elec = 0;
//                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
//                    Encoder_Begin(ENCODER_MOTOR_1);
//                }
//            }
//        }
        //����ͬʱʹ��
//        else if (checkMethod == CIRCLE_CHECK_BOTH) {
//            Circle_CheckCamera();
//            Circle_CheckElec();
//            if (Circle_leftBeginStatus_Camera == 1 && Circle_leftBeginStatus_Elec == 1) {
//                Circle_status = CIRCLE_LEFT_BEGIN;
//                Circle_leftBeginStatus_Camera = 0;
//                Circle_leftBeginStatus_Elec = 0;
//                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
//                    Encoder_Begin(ENCODER_MOTOR_2);
//                }
//            }
//            else if (Circle_rightBeginStatus_Camera == 1 && Circle_rightBeginStatus_Elec == 1) {
//                Circle_status = CIRCLE_RIGHT_BEGIN;
//                Circle_rightBeginStatus_Camera = 0;
//                Circle_rightBeginStatus_Elec = 0;
//                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
//                    Encoder_Begin(ENCODER_MOTOR_1);
//                }
//            }
//        }
    }

    //----------------------------------------
    //����Բ�����д���
    if (runMethod == CIRCLE_RUN_CAMERA) {
        Circle_RunCamera();
    }
//    else if (runMethod == CIRCLE_RUN_ELEC) {
//        Circle_RunElec(entryMethod);
//    }
//    else if (runMethod == CIRCLE_RUN_BOTH) {
//        Circle_RunBoth();
//    }
}
void Circle_RunGyscopAndEncoder(){

}

/*
 * xiao_barrier.c
 *
 *  Created on: 2024��4��4��
 *      Author: ��
 */
#include "xiao_barrier.h"
#include "xiao_gyroscope.h"
//------------------------------״̬��------------------------------
BARRIER_STATUS Barrier_status = BARRIER_NONE;

//------------------------------��������------------------------------
GYROSCOPE_MEASURE_TYPE Barrier_measureType = GYROSCOPE_GYRO_X;      //�ǶȲ���(��Ҫ����ʹ���ĸ�)
uint8 Barrier_usefulBarrier = 2;                                    //���������ϰ���
uint8 Barrier_barrierCnt = 0;                                       //�ϰ������
uint16 Barrier_encoderSumBridge_Thre = 60000;                       //�µ����ּ��


//--------------------����ϰ�--------------------
uint16 Barrier_distance_Thre = 1200;            //�ϰ���������ֵ (��λΪmm)
uint8 Barrier_distance_scCnt = 0;               //����������������
uint8 Barrier_distanceScCnt_Thre = 1;           //��������������ֵ
uint8 Barrier_distance_nscCnt = 0;              //������������������
uint8 Barrier_distanceNscCnt_Thre = 7;         //��������������������ֵ

//--------------------��ת����--------------------
int8 Barrier_speedVarTurnLeft_Motor1 = 10;      //��ת���ֵ��1�ٶȱ仯��
int8 Barrier_speedVarTurnLeft_Motor2 = -10;   //��ת���ֵ��2�ٶȱ任��
uint8 Barrier_forceAngleTurnLeft_Status = 0;    //��ת����ǿ�ƴ��״̬��
int16 Barrier_encoderSumTurnLeft_Thre = 3000;   //��ת������������ֵ (���ֵ��1)
float Barrier_AngleTurnLeft_Thre = 45.0;        //��ת�ǶȻ�����ֵ

//--------------------ֱ�߲���--------------------
int16 Barrier_encoderSumGoStraight_Thre = 5530; //ֱ�߲��ֱ�����������ֵ (���ֵ��1)

//--------------------��ת����--------------------
int8 Barrier_speedVarTurnRight_Motor1 = -10;    //��ת���ֵ��1�ٶȱ仯��
int8 Barrier_speedVarTurnRight_Motor2 = 10;     //��ת���ֵ��2�ٶȱ任��
uint8 Barrier_forceAngleTurnRight_Status = 0;   //��ת����ǿ�ƴ��״̬��
int16 Barrier_encoderSumTurnRight_Thre = 4000;  //��ת������������ֵ (���ֵ��2)
float Barrier_AngleTurnRight_Thre = 65.0;       //��ת�ǶȻ�����ֵ

//--------------------����--------------------
int16 Barrier_encoderSumGoStraightEnd_Thre = 10000; //��������ֱ�߱�����������ֵ (���ֵ��1)


/*
 * @brief               �ϰ����
 * @attention           ʹ�õ�ʱ����Ҫ�Ȱ�dl1a��ʼ��
 */
void Barrier_Check(void) {
    if (dl1a_finsh_flag == 1) {
        //��⴦��
        if (dl1a_distance_mm < Barrier_distance_Thre && Barrier_status == BARRIER_NONE) {
            Barrier_distance_nscCnt = 0;
            ++Barrier_distance_scCnt;
            if (Barrier_distance_scCnt > Barrier_distanceScCnt_Thre) {
                Barrier_distance_scCnt = 0;
                ++Barrier_barrierCnt;
                //Beep_Tweet();
                if (Barrier_barrierCnt != Barrier_usefulBarrier) {
                    Barrier_status = BARRIER_IS_BRIDGE;
                    Encoder_Begin(ENCODER_MOTOR_1);
                }
                else {
                    Barrier_status = BARRIER_BEGIN;
                }
            }
        }

        //���д���
        if (dl1a_distance_mm > Barrier_distance_Thre && Barrier_distance_scCnt != 0) {
            ++Barrier_distance_nscCnt;
            if (Barrier_distance_nscCnt > Barrier_distanceNscCnt_Thre) {
                Barrier_distance_nscCnt = 0;
                Barrier_distance_scCnt = 0;
                Barrier_status = BARRIER_NONE;
            }
        }
        dl1a_finsh_flag = 0;
    }
}


void Barrier_Run(void) {
    if (Barrier_status == BARRIER_BEGIN) {
     //   Elec_pidStatus = 0;
        Motor_1Target = Speed_set - 20 + Barrier_speedVarTurnLeft_Motor1;
        Motor_2Target = Speed_set - 20 + Barrier_speedVarTurnLeft_Motor2;
        Barrier_forceAngleTurnLeft_Status = 1;
        Gyroscope_Begin(Barrier_measureType);
        Barrier_status = BARRIER_TURN_LEFT;
    }
    else if (Barrier_status == BARRIER_TURN_LEFT) {
        if (Barrier_forceAngleTurnLeft_Status == 1) {
            if (fabs(Gyro_x) > Barrier_AngleTurnLeft_Thre) {
                Gyroscope_End(Barrier_measureType);
                Gyroscope_Clear(Barrier_measureType);
                Barrier_forceAngleTurnLeft_Status = 0;
                Motor_1Target = Speed_set - 20;
                Motor_2Target = Speed_set - 20;
                Encoder_Begin(ENCODER_MOTOR_1);
                Barrier_status = BARRIER_GO_STRAIGHT;
            }
        }
    }
    else if (Barrier_status == BARRIER_GO_STRAIGHT) {
        if (Encoder_sum_Motor1 > Barrier_encoderSumGoStraight_Thre) {
           // Beep_Tweet();
            Encoder_End(ENCODER_MOTOR_1);

            Motor_1Target = Speed_set - 20 + Barrier_speedVarTurnRight_Motor1;
            Motor_2Target = Speed_set - 20 + Barrier_speedVarTurnRight_Motor2;
            Barrier_forceAngleTurnRight_Status = 1;

            Gyroscope_Begin(Barrier_measureType);
            Barrier_status = BARRIER_TURN_RIGHT;
        }
    }
    else if (Barrier_status == BARRIER_TURN_RIGHT) {
        if (Barrier_forceAngleTurnRight_Status == 1) {
            if (fabs(Gyro_x) > Barrier_AngleTurnRight_Thre) {
                //Elec_pidStatus = 0;
                Gyroscope_End(Barrier_measureType);
                Barrier_forceAngleTurnRight_Status = 0;
                Motor_1Target = Speed_set - 20;
                Motor_2Target = Speed_set - 20;
                Encoder_Begin(ENCODER_MOTOR_2);
                Barrier_status = BARRIER_END;
            }
        }
    }
    else if (Barrier_status == BARRIER_END) {
        if (abs(Encoder_sum_Motor2)> Barrier_encoderSumGoStraightEnd_Thre) {
            Encoder_End(ENCODER_MOTOR_2);
            Motor_1Target = 0;
            Motor_2Target = 0;
//            Elec_pidStatus = 1;
            Barrier_status = BARRIER_NONE;
        }
    }
    else if (Barrier_status == BARRIER_IS_BRIDGE) {
        if (abs(Encoder_sum_Motor1) >= Barrier_encoderSumBridge_Thre) {
            Encoder_End(ENCODER_MOTOR_1);
            Encoder_Clear(Encoder_sum_Motor1);
            Barrier_status = BARRIER_NONE;
          //  Beep_Tweet();
        }
    }
}





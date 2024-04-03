/*
 * xiao_encoder.c
 *
 *  Created on: 2023��4��29��
 *      Author: Jayden_NaN
 */

#include "xiao_encoder.h"



//------------------------------------------------------------
//���״̬��
uint8 Encoder_readFinishStatus = 1;

//------------------------------��������------------------------------
//�������
int16 Encoder_1DataRead = 0;                        //���1�ɼ���������
int16 Encoder_2DataRead = 0;                        //���2�ɼ���������
int16 Encoder_leftFilter[ENCODER_FILTER_MAX + 1];   //�����˲����л�����
int16 Encoder_rightFilter[ENCODER_FILTER_MAX + 1];  //�����˲����л�����
int16 Encoder_dataPointer = 0;                      //��һ��ָ��,����ѭ������
int16 Encoder_1Data = 0;                            //���1�˲��������
int16 Encoder_2Data = 0;                            //���2�˲��������
//------------------------------
//�����˲�ʱ���Ȩ��
float Encoder_filterWeight[ENCODER_FILTER_MAX] = {0.8, 0.1, 0.06, 0.04};

//------------------------------�������------------------------------
int32 Encoder_sum_Motor1 = 0;                           //���1����
uint8 Encoder_sumStatus_Motor1 = 0;                     //���1����״̬
int32 Encoder_sum_Motor2 = 0;                           //���2����
uint8 Encoder_sumStatus_Motor2 = 0;                     //���2����״̬
//uint8 Encoder_sumStatus = 0;                          //��¼�����ж��Ƿ����

/**
 *  @brief  ��ʼ������������
 *  @return NULL
 */
void Encoder_Init(void) {
    //��������ʼ��
    encoder_dir_init(ENCODER_1_DIR, ENCODER_1_DIR_PULSE, ENCODER_1_DIR_DIR);
    encoder_dir_init(ENCODER_2_DIR, ENCODER_2_DIR_PULSE, ENCODER_2_DIR_DIR);
}

/**
 * @brief   ��������ȡһ�����ݲ������˲�
 * @return  NULL
 */
void Encoder_SpeedRead(void) {
    Encoder_readFinishStatus = 0;
    //���ݲɼ��Ĺ���
    Encoder_1DataRead = encoder_get_count(ENCODER_1_DIR);
    Encoder_2DataRead = encoder_get_count(ENCODER_2_DIR);
    encoder_clear_count(ENCODER_1_DIR);
    encoder_clear_count(ENCODER_2_DIR);
    //���������������Ľ��������� ->  �������ʾ������(�ܹ�)
    Encoder_1DataRead = -Encoder_1DataRead;
    Encoder_1Data = 0;
    Encoder_2Data = 0;
    //����һ���������ڵ��˲�
    Encoder_leftFilter[Encoder_dataPointer] = Encoder_1DataRead;
    Encoder_rightFilter[Encoder_dataPointer] = Encoder_2DataRead;
    for (int i = 0; i < ENCODER_FILTER_MAX; ++i) {
        int index = Encoder_dataPointer - i;
        if (index < 0)
            index += ENCODER_FILTER_MAX;
        Encoder_1Data += Encoder_leftFilter[index] * Encoder_filterWeight[i];
        Encoder_2Data += Encoder_rightFilter[index] * Encoder_filterWeight[i];
    }
    Encoder_dataPointer = (++Encoder_dataPointer) % ENCODER_FILTER_MAX;
    //������
    //Encoder_1Data = -Encoder_1Data;
    //Encoder_2Data = -Encoder_2Data;
    Encoder_readFinishStatus = 1;
}

/*
 * @brief                   ��������ʼ����
 * @parameter motorSelect   ���ѡ��
 * @example
 * @attention               ��������Ҫ���ж��н��е���
 */
void Encoder_Begin(ENCODER_MOTOR_SELECT motorSelect) {
    if (motorSelect == ENCODER_MOTOR_1) {
        if (Encoder_sumStatus_Motor1 == 0) {
            Encoder_sumStatus_Motor1 = 1;
            Encoder_sum_Motor1 = 0;
        }
    }
    else if (motorSelect == ENCODER_MOTOR_2) {
        if (Encoder_sumStatus_Motor2 == 0) {
            Encoder_sumStatus_Motor2 = 1;
            Encoder_sum_Motor2 = 0;
        }
    }
}

/*
 * @brief                   ������ֹͣ����
 * @parameter motorSelect   ���ѡ��
 * @example
 * @attention               ��������Ҫ���ж��н��е���
 */
void Encoder_End(ENCODER_MOTOR_SELECT motorSelect) {
    if (motorSelect == ENCODER_MOTOR_1) {
        Encoder_sumStatus_Motor1 = 0;
    }
    else if (motorSelect == ENCODER_MOTOR_2) {
        Encoder_sumStatus_Motor2 = 0;
    }
}

void Encoder_Count(void) {
    //------------------------------����˵��------------------------------
    //1. ���л���,���ж���,������ȡ���ݵĺ���

    //------------------------------��������------------------------------
    if (Encoder_readFinishStatus == 1) {
        if (Encoder_sumStatus_Motor1 == 1) {
            Encoder_sum_Motor1 += Encoder_1Data;
        }

        if (Encoder_sumStatus_Motor2 == 1) {
            Encoder_sum_Motor2 += Encoder_2Data;
        }
    }
}

/*
 * @brief               �������Ի��ֵ����ݽ�����0
 * @example
 */
void Encoder_Clear(ENCODER_MOTOR_SELECT motorSelect) {
    if (motorSelect == ENCODER_MOTOR_1) {
        Encoder_sum_Motor1 = 0;
    }
    else if (motorSelect == ENCODER_MOTOR_2) {
        Encoder_sum_Motor2 = 0;
    }
}

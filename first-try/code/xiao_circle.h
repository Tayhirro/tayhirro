/*
 * xiao_circle.h
 *
 *  Created on: 2023��7��2��
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_CIRCLE_H_
#define CODE_XIAO_CIRCLE_H_
#include "xiao_common.h"
#include "xiao_image_processing.h"
#include "xiao_trace.h"
#include "xiao_gyroscope.h"
#include "../user/cpu0_main.h"

//��������
typedef enum {
    CIRCLE_NONE = 0x00,

    CIRCLE_LEFT_BEGIN = 0x01,
    CIRCLE_LEFT_IN = 0x02,
    CIRCLE_LEFT_RUNNING = 0x03,
    CIRCLE_LEFT_OUT = 0x04,
    CIRCLE_LEFT_END = 0x05,

    CIRCLE_RIGHT_BEGIN = 0x11,
    CIRCLE_RIGHT_IN = 0x12,
    CIRCLE_RIGHT_RUNNING = 0x13,
    CIRCLE_RIGHT_OUT = 0x14,
    CIRCLE_RIGHT_END = 0x15,

    CIRCLE_ONLY_ONE = 0x20,
}CIRCLE_STATUS;

typedef enum {
    CIRCLE_CHECK_CAMERA = 0x00,             //��ʹ������ͷ�ж��뻷
    CIRCLE_CHECK_ELEC   = 0x01,             //��ʹ�õ���ж��뻷
    CIRCLE_CHECK_BOTH   = 0x02,             //���߾�ʹ��
}CIRCLE_CHECK_METHOD;

typedef enum {
    CIRCLE_RUN_CAMERA = 0x00,               //�ڻ��ڽ�ʹ������ͷ����Ѳ��
    CIRCLE_RUN_ELEC   = 0x01,               //�ڻ��ڽ�ʹ�õ�Ž���Ѳ��
    CIRCLE_RUN_BOTH   = 0x02,               //�ڻ��ڶ���ͬʱʹ��
}CIRCLE_RUN_METHOD;

typedef enum {
    CIRCLE_ENTRY_ENCODER = 0x00,                  //����������
    CIRCLE_ENTRY_ELEC_THRE = 0x01,                //��ֵ�ж�
    CIRCLE_ENTRY_CAMERA = 0x02,                   //����ͷ�ж�
}CIRCLE_PRE_ENTRY_METHOD;

//------------------------------�ⲿ�ӿ�------------------------------
extern CIRCLE_STATUS Circle_status;

//------------------------------_��������_------------------------------
extern int16 Circle_encoderLeft_Thre;            //�󻷵���������ֵ         -       ����:100
extern int16 Circle_encoderRight_Thre;           //�һ�����������ֵ         -       ����:100
extern uint8 Circle_multiCircle_Status;          //�໷��״̬��            -        0:ֻ��һ������;1:�ж������
extern uint8 Circle_speedAcc_Status;             //����������״̬��         -        0:������;1,����
//------------------------------_��������_------------------------------


void Circle_CheckCamera(void);
void Circle_RunElec(CIRCLE_PRE_ENTRY_METHOD entryMethod);
void Grage_Departure_Check(void);




#endif /* CODE_XIAO_CIRCLE_H_ */

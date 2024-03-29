/*
 * xiao_trace.h
 *
 *  Created on: 2023��7��2��
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_TRACE_H_
#define CODE_XIAO_TRACE_H_

//==============================��Ҫʹ�õ���ͷ�ļ�==============================
#include "xiao_common.h"
#define counter_number 2
//==============================��Ҫʹ�õ���ö��==============================
typedef enum {
TRACE_CENTERLINENEAR=0x00,   //��������
//--------------------------------------------------------------
TRACE_CENTERLINEFAR=0X01,    //ʮ��·��ʹ��
//--------------------------------------------------------------
TRACE_RIGHTLOST=0x02,   //���߶�ʧ//��pidoriginʱ��Ч//
TRACE_LEFTLOST=0x03,   //���߶�ʧ1//��pidoriginʱ��Ч//
TRACE_CROSS=0x04,   //ʮ��
TRACE_CIRCLE=0x05   //����
}TRACE_STATUS;




typedef enum {
    TRACE_Camera_MID = 0x01,
    TRACE_Camera_Near_LEFT = 0x02,      //ʹ������ͷѰ��������
    TRACE_Camera_Near_RIGHT = 0x03,     //ʹ������ͷѰ��������
//--------------------------------------------------------------
    TRACE_Camera_Near=0x04,         //���߲������Ѱ����Ѱ��ʱ
    TRACE_Camera_Far=0x05,          //Զ�߲������Ѱ����Ѱ��ʱ
    TRACE_Camera_Far_LEFT=0x06,
    TRACE_Camera_Far_RIGHT=0x07
}TRACE_TYPE;



typedef enum{
    TRACE_CROSS_CAREMA=0x00,        //��ʱû��
    TRACE_CROSS_CAREMA_GYROSCOPE_ENCODER=0x02,
    TRACE_CROSS_GYROSCOPE_ENCODER=0x03
}TRACE_CROSS_TYPE;






typedef enum{
        TRACE_CIRCLE_CAREMA=0x00,       //��ʱû��
        TRACE_CIRCLE_CAREMA_GYROSCOPE_ENCODER=0x02,
        TRACE_CIRCLE_GYROSCOPE_ENCODER=0x03
}TRACE_CIRCLE_TYPE;

extern TRACE_CIRCLE_TYPE Trace_Circle_Type;
extern TRACE_CROSS_TYPE Trace_Cross_Type;

extern TRACE_STATUS Trace_Status;
//==============================�ⲿ�����ӿ�==============================
extern TRACE_TYPE Trace_traceType;  //С��Ѱ������
extern int counter;
//==============================�ⲿ�����ӿ�==============================
void Trace_PIDInit();
void Trace_PID_Set(float K_p_set, float K_d_set, float coLimit, float boost, TRACE_TYPE traceType);
float Trace_Run();
//==============================vofa�ӿ�(����ɾ��)==============================
void Trace_SetPIDP(float setP, TRACE_TYPE traceType);
void Trace_SetPIDI(float setP, TRACE_TYPE traceType);
void Trace_SetPIDD(float setP, TRACE_TYPE traceType);
void Trace_SetPIDSumLimit(float sumLimit, TRACE_TYPE traceType);
void Trace_SetPIDCoLimit(float coLimit, TRACE_TYPE traceType);
extern float Trace_angleError;
extern float Trace_angleErrorTher ;               //�Ƕ������ֵ
extern float Trace_angleError_bak[counter_number];
extern uint8 Trace_aimLine ;
#endif /* CODE_XIAO_TRACE_H_ */

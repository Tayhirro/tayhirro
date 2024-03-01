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
#include "xiao_image_processing.h"
#include "xiao_motor.h"
#include "xiao_pid.h"

//==============================��Ҫʹ�õ���ö��==============================
typedef enum {
    TRACE_Elec = 0x01,              //ʹ�õ��Ѱ��
    TRACE_Camera_LEFT = 0x02,      //ʹ������ͷѰ����
    TRACE_Camera_RIGHT = 0x03,     //ʹ������ͷѰ����
}TRACE_TYPE;

//==============================�ⲿ�����ӿ�==============================
extern TRACE_TYPE Trace_traceType;  //С��Ѱ������

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
#endif /* CODE_XIAO_TRACE_H_ */

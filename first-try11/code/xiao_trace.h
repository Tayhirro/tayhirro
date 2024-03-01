/*
 * xiao_trace.h
 *
 *  Created on: 2023年7月2日
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_TRACE_H_
#define CODE_XIAO_TRACE_H_

//==============================需要使用到的头文件==============================
#include "xiao_common.h"
#include "xiao_image_processing.h"
#include "xiao_motor.h"
#include "xiao_pid.h"

//==============================需要使用到的枚举==============================
typedef enum {
    TRACE_Elec = 0x01,              //使用电磁寻迹
    TRACE_Camera_LEFT = 0x02,      //使用摄像头寻左线
    TRACE_Camera_RIGHT = 0x03,     //使用摄像头寻右线
}TRACE_TYPE;

//==============================外部变量接口==============================
extern TRACE_TYPE Trace_traceType;  //小车寻迹类型

//==============================外部函数接口==============================
void Trace_PIDInit();
void Trace_PID_Set(float K_p_set, float K_d_set, float coLimit, float boost, TRACE_TYPE traceType);
float Trace_Run();
//==============================vofa接口(不可删除)==============================
void Trace_SetPIDP(float setP, TRACE_TYPE traceType);
void Trace_SetPIDI(float setP, TRACE_TYPE traceType);
void Trace_SetPIDD(float setP, TRACE_TYPE traceType);
void Trace_SetPIDSumLimit(float sumLimit, TRACE_TYPE traceType);
void Trace_SetPIDCoLimit(float coLimit, TRACE_TYPE traceType);
#endif /* CODE_XIAO_TRACE_H_ */

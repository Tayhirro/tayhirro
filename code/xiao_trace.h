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
TRACE_CENTERLINENEAR=0x00,   //中线搜索
//--------------------------------------------------------------
TRACE_CENTERLINEFAR=0X01,    //十字路口使用
//--------------------------------------------------------------
TRACE_RIGHTLOST=0x02,   //右线丢失//在pidorigin时有效//
TRACE_LEFTLOST=0x03,   //左线丢失1//在pidorigin时有效//
TRACE_CROSS=0x04,   //十字
TRACE_CIRCLE=0x05   //环岛
}TRACE_STATUS;




typedef enum {
    TRACE_Camera_MID = 0x01,
    TRACE_Camera_LEFT = 0x02,      //使用摄像头寻近处左线
    TRACE_Camera_RIGHT = 0x03,     //使用摄像头寻近处右线
//--------------------------------------------------------------
    TRACE_Camera_Near=0x04,         //十字路口使用
    TRACE_Camera_Far=0x05,          //十字路口使用
    TRACE_Camera_Far_Both=0x06      //十字路口使用
}TRACE_TYPE;



typedef enum{
    TRACE_CROSS_CAREMA=0x00,        //暂时没用
    TRACE_CROSS_CAREMA_GYROSCOPE_ENCODER=0x02,
    TRACE_CROSS_GYROSCOPE_ENCODER=0x03
}TRACE_CROSS_TYPE;






typedef enum{
        TRACE_CIRCLE_CAREMA=0x00,       //暂时没用
        TRACE_CIRCLE_CAREMA_GYROSCOPE_ENCODER=0x02,
        TRACE_CIRCLE_GYROSCOPE_ENCODER=0x03
}TRACE_CIRCLE_TYPE;

extern TRACE_CIRCLE_TYPE Trace_Circle_Type;
extern TRACE_CROSS_TYPE Trace_Cross_Type;

extern TRACE_STATUS Trace_Status;
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
extern float Trace_angleError;
extern float Trace_angleErrorTher ;               //角度误差阈值
extern uint8 Trace_aimLine ;
extern fPID Trace_cameraLeftPID;                       //左边线获取的中线的PID
extern fPID Trace_cameraRightPID;                      //右边线获取的中线的PID
extern fPID Trace_cameraMidPID;                       //左+右获取的中线的PID
#endif /* CODE_XIAO_TRACE_H_ */

/*
 * xiao_circle.h
 *
 *  Created on: 2023年7月2日
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_CIRCLE_H_
#define CODE_XIAO_CIRCLE_H_
#include "xiao_common.h"
#include "xiao_image_processing.h"
#include "xiao_trace.h"
#include "xiao_gyroscope.h"
#include "../user/cpu0_main.h"

//环岛处理
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
    CIRCLE_CHECK_CAMERA = 0x00,             //仅使用摄像头判断入环
    CIRCLE_CHECK_ELEC   = 0x01,             //仅使用电磁判断入环
    CIRCLE_CHECK_BOTH   = 0x02,             //二者均使用
}CIRCLE_CHECK_METHOD;

typedef enum {
    CIRCLE_RUN_CAMERA = 0x00,               //在环内仅使用摄像头进行巡线
    CIRCLE_RUN_ELEC   = 0x01,               //在环内仅使用电磁进行巡线
    CIRCLE_RUN_BOTH   = 0x02,               //在环内二者同时使用
}CIRCLE_RUN_METHOD;

typedef enum {
    CIRCLE_ENTRY_ENCODER = 0x00,                  //编码器积分
    CIRCLE_ENTRY_ELEC_THRE = 0x01,                //阈值判断
    CIRCLE_ENTRY_CAMERA = 0x02,                   //摄像头判断
}CIRCLE_PRE_ENTRY_METHOD;

//------------------------------外部接口------------------------------
extern CIRCLE_STATUS Circle_status;

//------------------------------_处理数据_------------------------------
extern int16 Circle_encoderLeft_Thre;            //左环岛编码器阈值         -       步进:100
extern int16 Circle_encoderRight_Thre;           //右环岛编码器阈值         -       步进:100
extern uint8 Circle_multiCircle_Status;          //多环岛状态机            -        0:只有一个环岛;1:有多个环岛
extern uint8 Circle_speedAcc_Status;             //出环岛加速状态机         -        0:不加速;1,加速
//------------------------------_处理数据_------------------------------


void Circle_CheckCamera(void);
void Circle_RunElec(CIRCLE_PRE_ENTRY_METHOD entryMethod);
void Grage_Departure_Check(void);




#endif /* CODE_XIAO_CIRCLE_H_ */

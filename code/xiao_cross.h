/*
 * xiao_cross.h
 *
 *  Created on: 2024��3��3��
 *      Author: Tayhirro
 */

#ifndef CODE_XIAO_CROSS_H_
#define CODE_XIAO_CROSS_H_
#include "xiao_common.h"
#include "xiao_image_processing.h"
#include "xiao_trace.h"
#include "xiao_gyroscope.h"
#include "../user/cpu0_main.h"
#include "xiao_shift.h"

//ʮ�ִ���
typedef enum {
    CROSS_NONE = 0x00,
    CROSS_BEGIN = 0x01,
    CROSS_RUNNING = 0x02,
    CROSS_IN = 0x03,
    CROSS_IN2 =0x04,
    CROSS_BEGIN2=0x05,
    CROSS_RUNNING2=0x06
}CROSS_STATUS;



//------------------------------�ⲿ�ӿ�------------------------------
extern CROSS_STATUS Cross_status;


extern int16 Circle_encoderLeft_Thre;            //�󻷵���������ֵ         -       ����:100
extern int16 Circle_encoderRight_Thre;           //�һ�����������ֵ         -       ����:100



//------------------------------_��������_------------------------------
void Cross_CheckCamera(void);
void Cross_RunGyscopAndEncoder();
void Cross_RunCamera();
#endif /* CODE_XIAO_CROSS_H_ */

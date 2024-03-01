/*
 * xiao_barrier.h
 *
 *  Created on: 2023��7��2��
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_BARRIER_H_
#define CODE_XIAO_BARRIER_H_
#include "xiao_common.h"
#include "xiao_motor.h"
#include "../user/cpu0_main.h"
//#include "xiao_beep.h"
#include "xiao_gyroscope.h"
#include "xiao_encoder.h"
typedef enum {
    BARRIER_NONE = 0x00,
    BARRIER_BEGIN = 0x01,
    BARRIER_TURN_LEFT = 0x02,
    BARRIER_GO_STRAIGHT = 0x03,
    BARRIER_TURN_RIGHT = 0x04,
    BARRIER_END = 0x05,

    BARRIER_IS_BRIDGE = 0x10,
}BARRIER_STATUS;

extern BARRIER_STATUS Barrier_status;

//------------------------------_��������_------------------------------
extern uint16 Barrier_distance_Thre;                //�ϰ�������            -   ����:10
extern int8 Barrier_speedVarTurnLeft_Motor1;        //����ת���1�ٶȱ仯��     -   ����:1
extern int8 Barrier_speedVarTurnLeft_Motor2;        //����ת���2�ٶȱ仯��     -   ����:1
extern float Barrier_AngleTurnLeft_Thre;            //����ת�Ƕ���ֵ           -   ����:0.5
extern int16 Barrier_encoderSumGoStraight_Thre;     //������ֱ�߻�����ֵ        -   ����:100
extern int8 Barrier_speedVarTurnRight_Motor1;       //����ת���1�ٶȱ仯��     -   ����:1
extern int8 Barrier_speedVarTurnRight_Motor2;       //����ת���2�ٶȱ仯��     -   ����:1
extern float Barrier_AngleTurnRight_Thre;           //����ת�Ƕ���ֵ           -   ����:0.5
extern int16 Barrier_encoderSumGoStraightEnd_Thre;  //�������ֱ���������        -   ����500

extern uint8 Barrier_usefulBarrier;                 //�����ϰ�������           -   ����:1
extern uint16 Barrier_encoderSumBridge_Thre;        //�µ�����                -   ����:1000
//------------------------------_��������_------------------------------

void Barrier_Check(void);
void Barrier_Run(void);
void Barrier_PutStatus(void);

#endif /* CODE_XIAO_BARRIER_H_ */

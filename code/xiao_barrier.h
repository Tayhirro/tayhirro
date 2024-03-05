/*
 * xiao_barrier.h
 *
 *  Created on: 2023年7月2日
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

//------------------------------_处理数据_------------------------------
extern uint16 Barrier_distance_Thre;                //障碍距离检测            -   步进:10
extern int8 Barrier_speedVarTurnLeft_Motor1;        //向左转电机1速度变化量     -   步进:1
extern int8 Barrier_speedVarTurnLeft_Motor2;        //向左转电机2速度变化量     -   步进:1
extern float Barrier_AngleTurnLeft_Thre;            //向左转角度阈值           -   步进:0.5
extern int16 Barrier_encoderSumGoStraight_Thre;     //编码器直走积分阈值        -   步进:100
extern int8 Barrier_speedVarTurnRight_Motor1;       //向右转电机1速度变化量     -   步进:1
extern int8 Barrier_speedVarTurnRight_Motor2;       //向右转电机2速度变化量     -   步进:1
extern float Barrier_AngleTurnRight_Thre;           //向右转角度阈值           -   步进:0.5
extern int16 Barrier_encoderSumGoStraightEnd_Thre;  //结束部分编码器积分        -   步进500

extern uint8 Barrier_usefulBarrier;                 //可用障碍物数量           -   步进:1
extern uint16 Barrier_encoderSumBridge_Thre;        //坡道积分                -   步进:1000
//------------------------------_处理数据_------------------------------

void Barrier_Check(void);
void Barrier_Run(void);
void Barrier_PutStatus(void);

#endif /* CODE_XIAO_BARRIER_H_ */

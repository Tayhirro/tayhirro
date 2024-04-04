/*
 * xiao_barrier.c
 *
 *  Created on: 2024年4月4日
 *      Author: 康
 */
#include "xiao_barrier.h"
#include "xiao_gyroscope.h"
//------------------------------状态机------------------------------
BARRIER_STATUS Barrier_status = BARRIER_NONE;

//------------------------------基本变量------------------------------
GYROSCOPE_MEASURE_TYPE Barrier_measureType = GYROSCOPE_GYRO_X;      //角度采样(需要测试使用哪个)
uint8 Barrier_usefulBarrier = 2;                                    //可用有用障碍物
uint8 Barrier_barrierCnt = 0;                                       //障碍物计数
uint16 Barrier_encoderSumBridge_Thre = 60000;                       //坡道积分检测


//--------------------检测障碍--------------------
uint16 Barrier_distance_Thre = 1200;            //障碍检测距离阈值 (单位为mm)
uint8 Barrier_distance_scCnt = 0;               //满足条件次数计数
uint8 Barrier_distanceScCnt_Thre = 1;           //满足条件次数阈值
uint8 Barrier_distance_nscCnt = 0;              //不满足条件次数计数
uint8 Barrier_distanceNscCnt_Thre = 7;         //不满足条件次数计数阈值

//--------------------左转部分--------------------
int8 Barrier_speedVarTurnLeft_Motor1 = 10;      //左转部分电机1速度变化量
int8 Barrier_speedVarTurnLeft_Motor2 = -10;   //左转部分电机2速度变换量
uint8 Barrier_forceAngleTurnLeft_Status = 0;    //左转部分强制打角状态机
int16 Barrier_encoderSumTurnLeft_Thre = 3000;   //左转编码器积分阈值 (积分电机1)
float Barrier_AngleTurnLeft_Thre = 45.0;        //左转角度积分阈值

//--------------------直走部分--------------------
int16 Barrier_encoderSumGoStraight_Thre = 5530; //直走部分编码器积分阈值 (积分电机1)

//--------------------右转部分--------------------
int8 Barrier_speedVarTurnRight_Motor1 = -10;    //右转部分电机1速度变化量
int8 Barrier_speedVarTurnRight_Motor2 = 10;     //右转部分电机2速度变换量
uint8 Barrier_forceAngleTurnRight_Status = 0;   //右转部分强制打角状态机
int16 Barrier_encoderSumTurnRight_Thre = 4000;  //右转编码器积分阈值 (积分电机2)
float Barrier_AngleTurnRight_Thre = 65.0;       //右转角度积分阈值

//--------------------结束--------------------
int16 Barrier_encoderSumGoStraightEnd_Thre = 10000; //结束部分直走编码器积分阈值 (积分电机1)


/*
 * @brief               障碍检测
 * @attention           使用的时候需要先把dl1a初始化
 */
void Barrier_Check(void) {
    if (dl1a_finsh_flag == 1) {
        //检测处理
        if (dl1a_distance_mm < Barrier_distance_Thre && Barrier_status == BARRIER_NONE) {
            Barrier_distance_nscCnt = 0;
            ++Barrier_distance_scCnt;
            if (Barrier_distance_scCnt > Barrier_distanceScCnt_Thre) {
                Barrier_distance_scCnt = 0;
                ++Barrier_barrierCnt;
                //Beep_Tweet();
                if (Barrier_barrierCnt != Barrier_usefulBarrier) {
                    Barrier_status = BARRIER_IS_BRIDGE;
                    Encoder_Begin(ENCODER_MOTOR_1);
                }
                else {
                    Barrier_status = BARRIER_BEGIN;
                }
            }
        }

        //误判处理
        if (dl1a_distance_mm > Barrier_distance_Thre && Barrier_distance_scCnt != 0) {
            ++Barrier_distance_nscCnt;
            if (Barrier_distance_nscCnt > Barrier_distanceNscCnt_Thre) {
                Barrier_distance_nscCnt = 0;
                Barrier_distance_scCnt = 0;
                Barrier_status = BARRIER_NONE;
            }
        }
        dl1a_finsh_flag = 0;
    }
}


void Barrier_Run(void) {
    if (Barrier_status == BARRIER_BEGIN) {
     //   Elec_pidStatus = 0;
        Motor_1Target = Speed_set - 20 + Barrier_speedVarTurnLeft_Motor1;
        Motor_2Target = Speed_set - 20 + Barrier_speedVarTurnLeft_Motor2;
        Barrier_forceAngleTurnLeft_Status = 1;
        Gyroscope_Begin(Barrier_measureType);
        Barrier_status = BARRIER_TURN_LEFT;
    }
    else if (Barrier_status == BARRIER_TURN_LEFT) {
        if (Barrier_forceAngleTurnLeft_Status == 1) {
            if (fabs(Gyro_x) > Barrier_AngleTurnLeft_Thre) {
                Gyroscope_End(Barrier_measureType);
                Gyroscope_Clear(Barrier_measureType);
                Barrier_forceAngleTurnLeft_Status = 0;
                Motor_1Target = Speed_set - 20;
                Motor_2Target = Speed_set - 20;
                Encoder_Begin(ENCODER_MOTOR_1);
                Barrier_status = BARRIER_GO_STRAIGHT;
            }
        }
    }
    else if (Barrier_status == BARRIER_GO_STRAIGHT) {
        if (Encoder_sum_Motor1 > Barrier_encoderSumGoStraight_Thre) {
           // Beep_Tweet();
            Encoder_End(ENCODER_MOTOR_1);

            Motor_1Target = Speed_set - 20 + Barrier_speedVarTurnRight_Motor1;
            Motor_2Target = Speed_set - 20 + Barrier_speedVarTurnRight_Motor2;
            Barrier_forceAngleTurnRight_Status = 1;

            Gyroscope_Begin(Barrier_measureType);
            Barrier_status = BARRIER_TURN_RIGHT;
        }
    }
    else if (Barrier_status == BARRIER_TURN_RIGHT) {
        if (Barrier_forceAngleTurnRight_Status == 1) {
            if (fabs(Gyro_x) > Barrier_AngleTurnRight_Thre) {
                //Elec_pidStatus = 0;
                Gyroscope_End(Barrier_measureType);
                Barrier_forceAngleTurnRight_Status = 0;
                Motor_1Target = Speed_set - 20;
                Motor_2Target = Speed_set - 20;
                Encoder_Begin(ENCODER_MOTOR_2);
                Barrier_status = BARRIER_END;
            }
        }
    }
    else if (Barrier_status == BARRIER_END) {
        if (abs(Encoder_sum_Motor2)> Barrier_encoderSumGoStraightEnd_Thre) {
            Encoder_End(ENCODER_MOTOR_2);
            Motor_1Target = 0;
            Motor_2Target = 0;
//            Elec_pidStatus = 1;
            Barrier_status = BARRIER_NONE;
        }
    }
    else if (Barrier_status == BARRIER_IS_BRIDGE) {
        if (abs(Encoder_sum_Motor1) >= Barrier_encoderSumBridge_Thre) {
            Encoder_End(ENCODER_MOTOR_1);
            Encoder_Clear(Encoder_sum_Motor1);
            Barrier_status = BARRIER_NONE;
          //  Beep_Tweet();
        }
    }
}





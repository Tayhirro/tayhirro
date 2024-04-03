/*
 * xiao_encoder.c
 *
 *  Created on: 2023年4月29日
 *      Author: Jayden_NaN
 */

#include "xiao_encoder.h"



//------------------------------------------------------------
//电机状态机
uint8 Encoder_readFinishStatus = 1;

//------------------------------基础数据------------------------------
//电机数据
int16 Encoder_1DataRead = 0;                        //电机1采集到的数据
int16 Encoder_2DataRead = 0;                        //电机2采集到的数据
int16 Encoder_leftFilter[ENCODER_FILTER_MAX + 1];   //左轮滤波队列缓冲区
int16 Encoder_rightFilter[ENCODER_FILTER_MAX + 1];  //右轮滤波队列缓冲区
int16 Encoder_dataPointer = 0;                      //做一个指针,做个循环队列
int16 Encoder_1Data = 0;                            //电机1滤波后的数据
int16 Encoder_2Data = 0;                            //电机2滤波后的数据
//------------------------------
//数据滤波时候的权重
float Encoder_filterWeight[ENCODER_FILTER_MAX] = {0.8, 0.1, 0.06, 0.04};

//------------------------------电机积分------------------------------
int32 Encoder_sum_Motor1 = 0;                           //电机1积分
uint8 Encoder_sumStatus_Motor1 = 0;                     //电机1积分状态
int32 Encoder_sum_Motor2 = 0;                           //电机2积分
uint8 Encoder_sumStatus_Motor2 = 0;                     //电机2积分状态
//uint8 Encoder_sumStatus = 0;                          //记录本次中断是否积分

/**
 *  @brief  初始化两个编码器
 *  @return NULL
 */
void Encoder_Init(void) {
    //编码器初始化
    encoder_dir_init(ENCODER_1_DIR, ENCODER_1_DIR_PULSE, ENCODER_1_DIR_DIR);
    encoder_dir_init(ENCODER_2_DIR, ENCODER_2_DIR_PULSE, ENCODER_2_DIR_DIR);
}

/**
 * @brief   编码器读取一次数据并进行滤波
 * @return  NULL
 */
void Encoder_SpeedRead(void) {
    Encoder_readFinishStatus = 0;
    //数据采集的工作
    Encoder_1DataRead = encoder_get_count(ENCODER_1_DIR);
    Encoder_2DataRead = encoder_get_count(ENCODER_2_DIR);
    encoder_clear_count(ENCODER_1_DIR);
    encoder_clear_count(ENCODER_2_DIR);
    //这里是做编码器的矫正的问题 ->  否则会显示负数据(很怪)
    Encoder_1DataRead = -Encoder_1DataRead;
    Encoder_1Data = 0;
    Encoder_2Data = 0;
    //进行一个滑动窗口的滤波
    Encoder_leftFilter[Encoder_dataPointer] = Encoder_1DataRead;
    Encoder_rightFilter[Encoder_dataPointer] = Encoder_2DataRead;
    for (int i = 0; i < ENCODER_FILTER_MAX; ++i) {
        int index = Encoder_dataPointer - i;
        if (index < 0)
            index += ENCODER_FILTER_MAX;
        Encoder_1Data += Encoder_leftFilter[index] * Encoder_filterWeight[i];
        Encoder_2Data += Encoder_rightFilter[index] * Encoder_filterWeight[i];
    }
    Encoder_dataPointer = (++Encoder_dataPointer) % ENCODER_FILTER_MAX;
    //倒着跑
    //Encoder_1Data = -Encoder_1Data;
    //Encoder_2Data = -Encoder_2Data;
    Encoder_readFinishStatus = 1;
}

/*
 * @brief                   编码器开始计数
 * @parameter motorSelect   电机选择
 * @example
 * @attention               编码器需要在中断中进行调用
 */
void Encoder_Begin(ENCODER_MOTOR_SELECT motorSelect) {
    if (motorSelect == ENCODER_MOTOR_1) {
        if (Encoder_sumStatus_Motor1 == 0) {
            Encoder_sumStatus_Motor1 = 1;
            Encoder_sum_Motor1 = 0;
        }
    }
    else if (motorSelect == ENCODER_MOTOR_2) {
        if (Encoder_sumStatus_Motor2 == 0) {
            Encoder_sumStatus_Motor2 = 1;
            Encoder_sum_Motor2 = 0;
        }
    }
}

/*
 * @brief                   编码器停止计数
 * @parameter motorSelect   电机选择
 * @example
 * @attention               编码器需要在中断中进行调用
 */
void Encoder_End(ENCODER_MOTOR_SELECT motorSelect) {
    if (motorSelect == ENCODER_MOTOR_1) {
        Encoder_sumStatus_Motor1 = 0;
    }
    else if (motorSelect == ENCODER_MOTOR_2) {
        Encoder_sumStatus_Motor2 = 0;
    }
}

void Encoder_Count(void) {
    //------------------------------基本说明------------------------------
    //1. 进行积分,放中断里,丢到读取数据的后面

    //------------------------------处理数据------------------------------
    if (Encoder_readFinishStatus == 1) {
        if (Encoder_sumStatus_Motor1 == 1) {
            Encoder_sum_Motor1 += Encoder_1Data;
        }

        if (Encoder_sumStatus_Motor2 == 1) {
            Encoder_sum_Motor2 += Encoder_2Data;
        }
    }
}

/*
 * @brief               编码器对积分的数据进行清0
 * @example
 */
void Encoder_Clear(ENCODER_MOTOR_SELECT motorSelect) {
    if (motorSelect == ENCODER_MOTOR_1) {
        Encoder_sum_Motor1 = 0;
    }
    else if (motorSelect == ENCODER_MOTOR_2) {
        Encoder_sum_Motor2 = 0;
    }
}

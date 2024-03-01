/*
 * xiao_vofa.h
 *
 *  Created on: 2023年4月29日
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_VOFA_H_
#define CODE_XIAO_VOFA_H_

#include "xiao_common.h"
#include "stdarg.h"
#include "../user/cpu0_main.h"
#include "xiao_motor.h"
#include "xiao_pid.h"
//#include "xiao_elec.h"
extern uint8 Vofa_readStatusWired;         //有线传输读取数据的状态机
extern uint8 Vofa_readStatusWireLess;      //无线传输读取数据的状态机

typedef enum {
    VOFA_WIRELESS = 0x00,            //无线传输
    VOFA_WIRED = 0x01,               //有线传输
}VOFA_TRAN_DEVICE;

typedef enum {
    VOFA_INT = 0x00,
    VOFA_FLOAT = 0x01,
}VOFA_DATA_TYPE;

void Vofa_DataAnalyze(VOFA_TRAN_DEVICE device);
void Vofa_WireLessPrintf(VOFA_DATA_TYPE dataType, int num, ...);

#endif /* CODE_XIAO_VOFA_H_ */

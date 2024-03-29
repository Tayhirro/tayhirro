/*
 * xiao_shift.h
 *
 *  Created on: 2024年3月13日
 *      Author: Tayhirro
 */

#ifndef CODE_XIAO_SHIFT_H_
#define CODE_XIAO_SHIFT_H_
#include "xiao_common.h"
#include "../user/cpu0_main.h"
typedef enum{
    SHIFT_LEFT=0x00,
    SHIFT_RIGHT=0x01,
    SHIFT_DNONE=0x02,
    SHIFT_BOTH=0x03
}SHIFT_DIRECTION;
extern SHIFT_DIRECTION Shift_Direction;
extern uint8 none_rightshift_line;
extern uint8 none_leftshift_line;
extern int16 Shift_encoderLeft_Thre;              //左轮编码器积分阈值
extern int16 Shift_encoderRight_Thre;             //右轮编码器积分阈值
extern int16 Shift_encoderCross_Thre;             //过十字路口的积分阈值
void check_shiftroad();

#endif /* CODE_XIAO_SHIFT_H_ */

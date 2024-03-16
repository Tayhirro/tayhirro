/*
 * xiao_shift.h
 *
 *  Created on: 2024年3月13日
 *      Author: Tayhirro
 */

#ifndef CODE_XIAO_SHIFT_H_
#define CODE_XIAO_SHIFT_H_
#include "xiao_common.h"
#include "xiao_image_processing.h"
#include "xiao_trace.h"
#include "xiao_gyroscope.h"
#include "../user/cpu0_main.h"
typedef enum{
    SHIFT_NONE=0x00,
    SHIFT_BEGIN=0x01,
    SHIFT_RUNNING=0x02,
    SHIFT_END=0x03
}SHIFT_STATUS;

typedef enum{
    SHIFT_LEFT=0x00,
    SHIFT_RIGHT=0x01,
    SHIFT_DNONE=0x02,
    SHIFT_CROSS=0x03
}SHIFT_DIRECTION;

extern SHIFT_STATUS Shift_Status;
extern SHIFT_DIRECTION Shift_Direction;

extern int16 Shift_encoderLeft_Thre;              //左轮编码器积分阈值
extern int16 Shift_encoderRight_Thre;             //右轮编码器积分阈值
void check_shiftroad();
void handle_shiftroad_left(void);
void handle_shiftroad_right(void);
void handle_shiftroad_cross(void);


#endif /* CODE_XIAO_SHIFT_H_ */

/*
 * xiao_cross.h
 *
 *  Created on: 2024年3月3日
 *      Author: Tayhirro
 */

#ifndef CODE_XIAO_CROSS_H_
#define CODE_XIAO_CROSS_H_
#include "xiao_common.h"
#include "xiao_image_processing.h"
#include "xiao_trace.h"
#include "xiao_gyroscope.h"
#include "../user/cpu0_main.h"

//十字处理
typedef enum {
    CROSS_NONE = 0x00,
    CROSS_BEGIN = 0x01,
    CROSS_IN = 0x02,
    CROSS_RUNNING = 0x03,
    CROSS_OUT = 0x04,
    CROSS_END = 0x05,
}CROSS_STATUS;



//------------------------------外部接口------------------------------
extern CROSS_STATUS Cross_status;




//------------------------------_处理数据_------------------------------
void Cross_CheckCamera(void);


#endif /* CODE_XIAO_CROSS_H_ */

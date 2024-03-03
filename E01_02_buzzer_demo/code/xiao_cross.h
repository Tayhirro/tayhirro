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

//ʮ�ִ���
typedef enum {
    CROSS_NONE = 0x00,
    CROSS_BEGIN = 0x01,
    CROSS_IN = 0x02,
    CROSS_RUNNING = 0x03,
    CROSS_OUT = 0x04,
    CROSS_END = 0x05,
}CROSS_STATUS;



//------------------------------�ⲿ�ӿ�------------------------------
extern CROSS_STATUS Cross_status;




//------------------------------_��������_------------------------------
void Cross_CheckCamera(void);


#endif /* CODE_XIAO_CROSS_H_ */

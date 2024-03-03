/*
 * xiao_cross.c
 *
 *  Created on: 2024年3月3日
 *      Author: Tayhirro
 */
#include "xiao_cross.h"


//----------------------------------------
CROSS_STATUS Cross_status=CROSS_NONE;

//----------------------------------------
//十字检测状态
uint8 Cross_BeginStatus_Camera = 0;    //摄像头判断开始状态机
//uint8 Circle_leftBeginStatus_Elec = 0;      //电磁判断左环岛开始状态机
//uint8 Circle_rightBeginStatus_Elec = 0;     //电磁判断右环岛开始状态机

//----------------------------------------
//摄像头 - 判断边线是否消失
uint8 none_left_line_cross = 0,                   //判断左边线是否消失
        none_right_line_cross = 0;                //判断右边线是否消失
uint8 have_left_line_cross = 0,                   //判断是否存在左边线
        have_right_line_cross = 0;                //判断是否存在右边线
//----------------------------------------


/*
 * @brief               通过检测角点的值,判断是否为十字路口
 * @attention           使用摄像头检测,需要对摄像头进行初始化
 */

void Cross_CheckCamera(void) {
    //十字
    if (Cross_status == CROSS_NONE && Image_LptLeft_Found && Image_LptRight_Found) {
        //测试代码
        Circle_status = CROSS_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_2);
//        put_int32(70, 1);
        gpio_set_level(P20_8, GPIO_LOW);
        gpio_set_level(P20_9, GPIO_LOW);
    }
}




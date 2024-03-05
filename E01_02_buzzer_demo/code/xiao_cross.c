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




//------------刚进入十字路口---------------------//
//摄像头 - 环岛大于阈值次数计数 (sc - satisfy condition)
uint8 Cross_camera_scCnt = 0;                  //十字判断摄像头部分满足次数
const uint8 Cross_camera_scCnt_Thre = 0;       //十字判断摄像头满足次数阈值(次数大于阈值就判断预入环)
uint8 Cross_camera_nscCnt = 0;                 //十字判断摄像头部分不满足次数
const uint8 Cross_camera_nscCnt_Thre = 0;      //环岛判断摄像头部分不满足次数阈值(次数大于阈值就判断非环岛)

//------------------------------预入十字部分------------------------------
int16 Cross_encoderLeft = 0;                       //左轮编码器积分
int16 Cross_encoderRight = 0;                      //右轮编码器积分
//----------------------------------------
uint8 Cross_forceAngle_Status = 0;             //十字路口强制打角状态机



//陀螺仪测量角度的时候使用的变量 - (假定先使用x轴陀螺仪)
GYROSCOPE_MEASURE_TYPE Cross_measureType = GYROSCOPE_GYRO_X;

/*
 * @brief               通过检测角点的值,判断是否为十字路口
 * @attention           使用摄像头检测,需要对摄像头进行初始化
 */

void Cross_CheckCamera(void) {
    //十字
    if (Cross_status == CROSS_NONE && Image_LptLeft_Found && Image_LptRight_Found) {
        //测试代码
        Cross_status = CROSS_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_2);
//        put_int32(70, 1);
        gpio_set_level(P20_8, GPIO_LOW);
        gpio_set_level(P20_9, GPIO_LOW);
    }
}
void Cross_RunCamera(){
    if (Cross_status == CROSS_BEGIN) {
            Trace_traceType = TRACE_Camera_Near;

            //两边线全都丢失
            if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
            if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
                if (none_left_line_cross>2&&none_right_line_cross>2) {
                    Cross_status = CROSS_RUNNING;
                    none_left_line_cross = 0;
                    none_right_line_cross = 0;
                    Gyroscope_Begin(Cross_measureType);
                }
            //当角点较近时，切换CROSS_RUNNING模式  //此时进行同时近线远线搜索，远线主导搜索//或者编码器积分//或者同时辅助
            else if (Cross_status == CROSS_RUNNING) {
                        Trace_traceType = TRACE_Camera_Far;
                        //----------------------------------------
                        //陀螺仪计数
                        if (Cross_measureType == GYROSCOPE_GYRO_X) {
                            //这里暂定为>= 90.0,实际的测量可能是负值,这里需要做一下改变
//                            if (Gyro_x >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
//                                Circle_status = CROSS_RUNNING;
//                                Gyroscope_End(Circle_measureType);
//                            }

                                if(1){           //近线重新找到，远线也找到
                                    Cross_status=CROSS_OUT;


                                }


                        }
                        else if (Cross_measureType == GYROSCOPE_GYRO_Y) {
//                            if (Gyro_y >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
//                                Circle_status = CROSS_RUNNING;
//                                Gyroscope_End(Circle_measureType);
//                            }
                            if(1){           //近线重新找到，远线也找到
                                                                Cross_status=CROSS_OUT;


                                                            }
                        }
                        else if (Cross_measureType == GYROSCOPE_GYRO_Z) {
//                            if (Gyro_z >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
//                                Circle_status = CIRCLE_RIGHT_RUNNING;
//                                Gyroscope_End(Circle_measureType);
//                            }
                            if(1){           //近线重新找到，远线也找到
                                                                                            Cross_status=CROSS_OUT;


                                                                                        }
                        }
                    }
        }


}



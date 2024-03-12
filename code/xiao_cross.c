/*
 * xiao_cross.c
 *
 *  Created on: 2024年3月3日
 *      Author: Tayhirro
 */
#include "xiao_cross.h"


//----------------------------------------
//----------------------------------------



CROSS_STATUS Cross_status=CROSS_NONE;
int16 Cross_encoderLeft_Thre = 15500;              //左轮编码器积分阈值
int16 Cross_encoderRight_Thre = 15500;             //右轮编码器积分阈值
uint8 Cross_forceAngle_Status = 0;             //十字路口强制打角状态机
//----------------------------------------
uint8 Cross_SteerVar    =   -13;    //打角值
uint8 Cross_Targetangle;            //目标角度值
uint8 Cross_Steer_Angle;        //外部
//----------------------------------------
//十字检测状态
uint8 Cross_BeginStatus_Camera = 0;    //摄像头判断开始状态机
//uint8 Cross_leftBeginStatus_Elec = 0;      //电磁判断左开始状态机
//uint8 Cross_rightBeginStatus_Elec = 0;     //电磁判断右开始状态机

//----------------------------------------
//摄像头 - 判断边线是否消失
uint8 none_left_line_cross = 0,                   //判断左边线是否消失
        none_right_line_cross = 0;                //判断右边线是否消失
uint8 have_left_line_cross = 0,                   //判断是否存在左边线
        have_right_line_cross = 0;                //判断是否存在右边线
//------------------------------预入十字部分------------------------------
int16 Cross_encoderLeft = 0;                       //左轮编码器积分
int16 Cross_encoderRight = 0;                      //右轮编码器积分
//------------------------------入十字部分------------------------------
//------------刚进入十字路口---------------------//
//摄像头 - 环岛大于阈值次数计数 (sc - satisfy condition)
uint8 Cross_camera_scCnt = 0;                  //十字判断摄像头部分满足次数
const uint8 Cross_camera_scCnt_Thre = 0;       //十字判断摄像头满足次数阈值(次数大于阈值就判断预入环)
uint8 Cross_camera_nscCnt = 0;                 //十字判断摄像头部分不满足次数
const uint8 Cross_camera_nscCnt_Thre = 0;      //环岛判断摄像头部分不满足次数阈值(次数大于阈值就判断非环岛)
//----------------------------------------
float Cross_angleEntry_Thre = 45.0;            //入十字角度积分阈值


//陀螺仪测量角度的时候使用的变量 - (假定先使用x轴陀螺仪)
GYROSCOPE_MEASURE_TYPE Cross_measureType = GYROSCOPE_GYRO_X;

/*
 * @brief               通过检测角点的值,判断是否为十字路口
 * @attention           使用摄像头检测,需要对摄像头进行初始化
 */

void Cross_CheckCamera(void) {
    //十字
    if (Trace_Status==TRACE_CENTERLINENEAR&&Cross_status == CROSS_NONE && Image_LptLeft_Found && Image_LptRight_Found) {
        //测试代码
        Cross_status = CROSS_BEGIN;
        Trace_Status = TRACE_CROSS;
        Encoder_Begin(ENCODER_MOTOR_1);
//        put_int32(70, 1);
        gpio_set_level(P20_8, GPIO_LOW);
        gpio_set_level(P20_9, GPIO_LOW);
    }
}



//---------------------Cross巡线---编码器积分和陀螺仪
//void Cross_RunCamera2(){
//    if (Cross_status == CROSS_BEGIN) {
//            Trace_traceType = TRACE_Camera_Near;
//    }
//
//}
void Cross_RunGyscopAndEncoder(){
    if (Cross_status == CROSS_BEGIN) {
        Trace_Status=TRACE_CROSS;
        Trace_traceType = TRACE_Camera_Near;


                //当两侧角点较近的时候，开启running模式//或者丢线时
               // if(Image_LptLeft_Found&&Image_LptRight_Found){
                  // Cross_status = CROSS_RUNNING;
                  // Trace_traceType = TRACE_Camera_Far_Both;
                //}
                   if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
                                if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
                                    if (none_left_line_cross>2&&none_right_line_cross>2) {
                                        none_left_line_cross = 0;
                                        none_right_line_cross = 0;
                                        Cross_status = CROSS_RUNNING;
                                        Trace_traceType = TRACE_Camera_Far_Both;
                                        Gyroscope_Begin(Cross_measureType);     //开启陀螺仪
                                    }
            //------------------------------------------------------------------------------------
            //------------------------------------------------------------------------------------
                        //循迹初始行向上移动
            //------------------------------------------------------------------------------------
            //------------------------------------------------------------------------------------
            //------------------------------------------------------------------------------------
               //
        }
                //当角点较近时，切换CROSS_RUNNING模式  //此时进行同时近线远线搜索，远线主导搜索//或者编码器积分//或者陀螺仪积分//或者同时辅助
                else if (Cross_status == CROSS_RUNNING) {
                              //近处搜寻到线
                            if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //可以修改||
                                Encoder_End(ENCODER_MOTOR_1);
                                Encoder_Clear(ENCODER_MOTOR_1);
                                Trace_traceType=TRACE_Camera_Near;      //只寻近线
                                Cross_status=CROSS_IN;
                            }




                                //----------------------------------------
                }

                else if(Cross_status==CROSS_IN){        //进入十字路口的弯道部分
                                                //陀螺仪方式来判断是否到达最终阶段
                                        if (Cross_measureType == GYROSCOPE_GYRO_X) {
                                                                               if (fabs(Gyro_x) > Cross_angleEntry_Thre) {
                                                                                   //进入环岛
                                                                                   Cross_forceAngle_Status = 0;
                                                                                   Elec_pidStatus = 1;
                                                                                   Cross_status = CROSS_IN2;


                                                                                   Cross_Steer_Angle=Cross_Targetangle+Cross_SteerVar;  //参数传递//会更改


                                                                                   Gyroscope_End(Cross_measureType);
                                                                                   Gyroscope_Clear(Cross_measureType);
                                                                                   Gyroscope_Begin(Cross_measureType);
                                                                               }
                                                                           }
                                                                           else if (Cross_measureType == GYROSCOPE_GYRO_Y) {
                                                                               if (fabs(Gyro_y) > Cross_angleEntry_Thre) {
                                                                                   //进入环岛
                                                                                   Cross_forceAngle_Status = 0;
                                                                                   Elec_pidStatus = 1;
                                                                                   Cross_status = CROSS_IN2;
                                                                                   Gyroscope_End(Cross_measureType);
                                                                                   Gyroscope_Clear(Cross_measureType);
                                                                                   Gyroscope_Begin(Cross_measureType);
                                                                               }
                                                                           }
                                                                           else if (Cross_measureType == GYROSCOPE_GYRO_Z) {
                                                                               if (fabs(Gyro_z) > Cross_angleEntry_Thre) {
                                                                                   //进入环岛
                                                                                   Cross_forceAngle_Status = 0;
                                                                                   Elec_pidStatus = 1;
                                                                                   Speed_set = 30;
                                                                                   Cross_status = CROSS_IN2;
                                                                                   Gyroscope_End(Cross_measureType);
                                                                                   Gyroscope_Clear(Cross_measureType);
                                                                                   Gyroscope_Begin(Cross_measureType);
                                                                               }
                                                                           }

                                    }







                     else if (Cross_status==CROSS_IN2) {
                                 //开始搜索左线，跟着左线的中线走,同时最后用fabs(Gyro_)来判断是否到达cross_begin_2阶段



                                if((Image_LptLeft_Found &&!Image_LptRight_Found)||(Image_LptLeft_Found&&Image_LptRight_Found)){          //左角点发现之后
                                                Cross_status=CROSS_BEGIN2;
                                                Encoder_Begin(ENCODER_MOTOR_2);
                                           }
                            }
                     else if (Cross_status==CROSS_BEGIN2) {

                         if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
                                                     if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
                                                         if (none_left_line_cross>2&&none_right_line_cross>2) {
                                                             none_left_line_cross = 0;
                                                             none_right_line_cross = 0;
                                                             Cross_status = CROSS_RUNNING2;
                                                             Trace_traceType = TRACE_Camera_Far;
                                                             Gyroscope_Begin(Cross_measureType);     //开启陀螺仪
                                                         }
                                             }
                     else if(Cross_status==CROSS_RUNNING2){
                         if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //可以修改||
                                                     Encoder_End(ENCODER_MOTOR_1);
                                                     Encoder_Clear(ENCODER_MOTOR_1);
                                                     Trace_traceType=TRACE_Camera_Near;
                                                     Cross_status=CROSS_NONE;
                                                     Trace_Status=TRACE_CENTERLINENEAR;
                                                 }
                     }

}
void Cross_RunCamera(){
    if (Cross_status == CROSS_BEGIN) {
            Trace_traceType = TRACE_Camera_Near;

            //当两侧角点较近的时候，开启running模式//或者丢线时
           // if(Image_LptLeft_Found&&Image_LptRight_Found){
              // Cross_status = CROSS_RUNNING;
              // Trace_traceType = TRACE_Camera_Far_Both;
            //}
               if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
                            if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
                                if (none_left_line_cross>2&&none_right_line_cross>2) {
                                    none_left_line_cross = 0;
                                    none_right_line_cross = 0;
                                    Cross_status = CROSS_RUNNING;
                                    Trace_traceType=TRACE_Camera_Far_Both;
                                    Gyroscope_Begin(Cross_measureType);     //开启陀螺仪
                                }
    }
            //当角点较近时，切换CROSS_RUNNING模式  //此时进行同时近线远线搜索，远线主导搜索//或者编码器积分//或者陀螺仪积分//或者同时辅助
            else if (Cross_status == CROSS_RUNNING) {
                          //近处搜寻到线
                        if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //可以修改||
                            Encoder_End(ENCODER_MOTOR_1);
                            Encoder_Clear(ENCODER_MOTOR_1);
                            Trace_traceType=TRACE_Camera_Near;      //只寻近线
                            Cross_status=CROSS_IN;
                        }




                            //----------------------------------------
            }

            else if(Cross_status==CROSS_IN){        //进入十字路口的弯道部分
                                            //陀螺仪方式来判断是否到达最终阶段
                                    if (Cross_measureType == GYROSCOPE_GYRO_X) {
                                                                           if (fabs(Gyro_x) > Cross_angleEntry_Thre) {
                                                                               //进入环岛
                                                                               Cross_forceAngle_Status = 0;
                                                                               Elec_pidStatus = 1;
                                                                               Cross_status = CROSS_IN2;


                                                                               Cross_Steer_Angle=Cross_Targetangle+Cross_SteerVar;  //参数传递//会更改


                                                                               Gyroscope_End(Cross_measureType);
                                                                               Gyroscope_Clear(Cross_measureType);
                                                                               Gyroscope_Begin(Cross_measureType);
                                                                           }
                                                                       }
                                                                       else if (Cross_measureType == GYROSCOPE_GYRO_Y) {
                                                                           if (fabs(Gyro_y) > Cross_angleEntry_Thre) {
                                                                               //进入环岛
                                                                               Cross_forceAngle_Status = 0;
                                                                               Elec_pidStatus = 1;
                                                                               Cross_status = CROSS_IN2;
                                                                               Gyroscope_End(Cross_measureType);
                                                                               Gyroscope_Clear(Cross_measureType);
                                                                               Gyroscope_Begin(Cross_measureType);
                                                                           }
                                                                       }
                                                                       else if (Cross_measureType == GYROSCOPE_GYRO_Z) {
                                                                           if (fabs(Gyro_z) > Cross_angleEntry_Thre) {
                                                                               //进入环岛
                                                                               Cross_forceAngle_Status = 0;
                                                                               Elec_pidStatus = 1;
                                                                               Speed_set = 30;
                                                                               Cross_status = CROSS_IN2;
                                                                               Gyroscope_End(Cross_measureType);
                                                                               Gyroscope_Clear(Cross_measureType);
                                                                               Gyroscope_Begin(Cross_measureType);
                                                                           }
                                                                       }

                                }







                 else if (Cross_status==CROSS_IN2) {
                             //开始搜索左线，跟着左线的中线走,同时最后用fabs(Gyro_)来判断是否到达cross_begin_2阶段



                            if((Image_LptLeft_Found &&!Image_LptRight_Found)||(Image_LptLeft_Found&&Image_LptRight_Found)){          //左角点发现之后
                                            Cross_status=CROSS_BEGIN2;
                                            Encoder_Begin(ENCODER_MOTOR_2);
                                       }
                        }
                 else if (Cross_status==CROSS_BEGIN2) {

                     if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
                                                 if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
                                                     if (none_left_line_cross>2&&none_right_line_cross>2) {
                                                         none_left_line_cross = 0;
                                                         none_right_line_cross = 0;
                                                         Cross_status = CROSS_RUNNING2;
                                                         Trace_traceType = TRACE_Camera_Far;
                                                         Gyroscope_Begin(Cross_measureType);     //开启陀螺仪
                                                     }
                                         }
                 else if(Cross_status==CROSS_RUNNING2){
                     if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //可以修改||
                                                 Encoder_End(ENCODER_MOTOR_1);
                                                 Encoder_Clear(ENCODER_MOTOR_1);
                                                 Trace_traceType=TRACE_Camera_Near;
                                                 Cross_status=CROSS_NONE;
                                                 Trace_Status=TRACE_CENTERLINENEAR;
                                             }
                 }
}



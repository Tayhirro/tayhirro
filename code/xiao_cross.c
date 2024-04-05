/*
 * xiao_cross.c
 *
 *  Created on: 2024年3月3日
 *      Author: Tayhirro
 */
#include "xiao_cross.h"
#include "xiao_encoder.h"
#include "xiao_image_processing.h"
#include "xiao_trace.h"
#include "xiao_gyroscope.h"
#include "xiao_shift.h"
#include "xiao_pid.h"
//----------------------------------------
//----------------------------------------



CROSS_STATUS Cross_status=CROSS_NONE;
//----------------------------------------
uint8 Cross_SteerVar    =   -13;    //打角值
uint8 Cross_Targetangle;            //目标角度值
uint8 Cross_Steer_Angle;        //外部
//------------------------------入十字部分------------------------------
//摄像头 - 环岛大于阈值次数计数 (sc - satisfy condition)
uint8 Cross_camera_scCnt = 0;                  //十字判断摄像头部分满足次数
const uint8 Cross_camera_scCnt_Thre = 0;       //十字判断摄像头满足次数阈值(次数大于阈值就判断预入环)
uint8 Cross_camera_nscCnt = 0;                 //十字判断摄像头部分不满足次数
const uint8 Cross_camera_nscCnt_Thre = 0;      //环岛判断摄像头部分不满足次数阈值(次数大于阈值就判断非环岛)
//----------------------------------------
float Cross_angleEntry_Thre = 180.0;            //入十字角度积分阈值
int16 EncoderCross_In_Thre=1500;
int16 EncoderCross_Thre=10000;
int32 EncoderCross_RUNNING_Thre=64000;
int16 EncoderCross_IN2_Thre=5000;
int16 Gyroscope_z_Cross_in_Thre=10.0;
int16 Gyroscope_z_Cross_running_Thre=280.0;
//陀螺仪测量角度的时候使用的变量 - (假定先使用x轴陀螺仪)
GYROSCOPE_MEASURE_TYPE Cross_measureType = GYROSCOPE_GYRO_Z;

/*
 * @brief               通过检测角点的值,判断是否为十字路口
 * @attention           使用摄像头检测,需要对摄像头进行初始化
 */

void Cross_CheckCamera(void) {
    //十字
    if (Trace_Status==TRACE_CENTERLINENEAR&& Image_LptLeft_Found&&Cross_status == CROSS_NONE) {//Cross_status == CROSS_NONE
        //测试代码
        Trace_Status=TRACE_CROSS;
        Cross_status = CROSS_BEGIN;
        Encoder_End(ENCODER_MOTOR_2);
        Encoder_Clear(ENCODER_MOTOR_2);
        Encoder_End(ENCODER_MOTOR_1);
        Encoder_Clear(ENCODER_MOTOR_1);
        Encoder_Begin(ENCODER_MOTOR_1);
        Encoder_Begin(ENCODER_MOTOR_2);
        Gyroscope_Begin(Cross_measureType);     //开启陀螺仪
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
//void Cross_RunGyscopAndEncoder(){
//    if (Cross_status == CROSS_BEGIN) {
//        Trace_Status=TRACE_CROSS;
//        Trace_traceType = TRACE_Camera_Near;
//
//
//                //当两侧角点较近的时候，开启running模式//或者丢线时
//               // if(Image_LptLeft_Found&&Image_LptRight_Found){
//                  // Cross_status = CROSS_RUNNING;
//                  // Trace_traceType = TRACE_Camera_Far_Both;
//                //}
//                   if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
//                                if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
//                                    if (none_left_line_cross>2&&none_right_line_cross>2) {
//                                        none_left_line_cross = 0;
//                                        none_right_line_cross = 0;
//                                        Cross_status = CROSS_RUNNING;
//                                        Trace_traceType = TRACE_Camera_Far_Both;
//                                        Gyroscope_Begin(Cross_measureType);     //开启陀螺仪
//                                    }
//            //------------------------------------------------------------------------------------
//            //------------------------------------------------------------------------------------
//                        //循迹初始行向上移动
//            //------------------------------------------------------------------------------------
//            //------------------------------------------------------------------------------------
//            //------------------------------------------------------------------------------------
//               //
//        }
//                //当角点较近时，切换CROSS_RUNNING模式  //此时进行同时近线远线搜索，远线主导搜索//或者编码器积分//或者陀螺仪积分//或者同时辅助
//                else if (Cross_status == CROSS_RUNNING) {
//                              //近处搜寻到线
//                            if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //可以修改||
//                                Encoder_End(ENCODER_MOTOR_1);
//                                Encoder_Clear(ENCODER_MOTOR_1);
//                                Trace_traceType=TRACE_Camera_Near;      //只寻近线
//                                Cross_status=CROSS_IN;
//                            }
//
//
//
//
//                                //----------------------------------------
//                }
//
//                else if(Cross_status==CROSS_IN){        //进入十字路口的弯道部分
//                                                //陀螺仪方式来判断是否到达最终阶段
//                                        if (Cross_measureType == GYROSCOPE_GYRO_X) {
//                                                                               if (fabs(Gyro_x) > Cross_angleEntry_Thre) {
//                                                                                   //进入环岛
//                                                                                   Cross_forceAngle_Status = 0;
//                                                                                   Elec_pidStatus = 1;
//                                                                                   Cross_status = CROSS_IN2;
//
//
//                                                                                   Cross_Steer_Angle=Cross_Targetangle+Cross_SteerVar;  //参数传递//会更改
//
//
//                                                                                   Gyroscope_End(Cross_measureType);
//                                                                                   Gyroscope_Clear(Cross_measureType);
//                                                                                   Gyroscope_Begin(Cross_measureType);
//                                                                               }
//                                                                           }
//                                                                           else if (Cross_measureType == GYROSCOPE_GYRO_Y) {
//                                                                               if (fabs(Gyro_y) > Cross_angleEntry_Thre) {
//                                                                                   //进入环岛
//                                                                                   Cross_forceAngle_Status = 0;
//                                                                                   Elec_pidStatus = 1;
//                                                                                   Cross_status = CROSS_IN2;
//                                                                                   Gyroscope_End(Cross_measureType);
//                                                                                   Gyroscope_Clear(Cross_measureType);
//                                                                                   Gyroscope_Begin(Cross_measureType);
//                                                                               }
//                                                                           }
//                                                                           else if (Cross_measureType == GYROSCOPE_GYRO_Z) {
//                                                                               if (fabs(Gyro_z) > Cross_angleEntry_Thre) {
//                                                                                   //进入环岛
//                                                                                   Cross_forceAngle_Status = 0;
//                                                                                   Elec_pidStatus = 1;
//                                                                                   Speed_set = 30;
//                                                                                   Cross_status = CROSS_IN2;
//                                                                                   Gyroscope_End(Cross_measureType);
//                                                                                   Gyroscope_Clear(Cross_measureType);
//                                                                                   Gyroscope_Begin(Cross_measureType);
//                                                                               }
//                                                                           }
//
//                                    }
//                     else if (Cross_status==CROSS_IN2) {
//                                 //开始搜索左线，跟着左线的中线走,同时最后用fabs(Gyro_)来判断是否到达cross_begin_2阶段
//
//                                if((Image_LptLeft_Found &&!Image_LptRight_Found)||(Image_LptLeft_Found&&Image_LptRight_Found)){          //左角点发现之后
//                                                Cross_status=CROSS_BEGIN2;
//                                                Encoder_Begin(ENCODER_MOTOR_2);
//                                           }
//                            }
//                     else if (Cross_status==CROSS_BEGIN2) {
//
//                         if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
//                                                     if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
//                                                         if (none_left_line_cross>2&&none_right_line_cross>2) {
//                                                             none_left_line_cross = 0;
//                                                             none_right_line_cross = 0;
//                                                             Cross_status = CROSS_RUNNING2;
//                                                             Trace_traceType = TRACE_Camera_Far;
//                                                             Gyroscope_Begin(Cross_measureType);     //开启陀螺仪
//                                                         }
//                                             }
//                     else if(Cross_status==CROSS_RUNNING2){
//                         if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //可以修改||
//                                                     Encoder_End(ENCODER_MOTOR_2);
//                                                     Encoder_Clear(ENCODER_MOTOR_2);
//                                                     Trace_traceType=TRACE_Camera_Near;
//                                                     Cross_status=CROSS_NONE;
//                                                     Trace_Status=TRACE_CENTERLINENEAR;
//                                                 }
//                     }
//
//}
//void Cross_RunCamera(){
//    if (Cross_status == CROSS_BEGIN) {
//            Trace_traceType = TRACE_Camera_Near;
//
//            //当两侧角点较近的时候，开启running模式//或者丢线时
//           // if(Image_LptLeft_Found&&Image_LptRight_Found){
//              // Cross_status = CROSS_RUNNING;
//              // Trace_traceType = TRACE_Camera_Far_Both;
//            //}
//               if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
//                            if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
//                                if (none_left_line_cross>2&&none_right_line_cross>2) {
//                                    none_left_line_cross = 0;
//                                    none_right_line_cross = 0;
//                                    Cross_status = CROSS_RUNNING;
//                                    Trace_traceType=TRACE_Camera_Near;
//                                    Gyroscope_Begin(Cross_measureType);     //开启陀螺仪
//                                }
//    }
//            //当角点较近时，切换CROSS_RUNNING模式  //此时进行同时近线远线搜索，远线主导搜索//或者编码器积分//或者陀螺仪积分//或者同时辅助
//            else if (Cross_status == CROSS_RUNNING) {
//                          //近处搜寻到线
//                        if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //可以修改||
//                            Encoder_End(ENCODER_MOTOR_2);
//                            Encoder_Clear(ENCODER_MOTOR_2);
//                            Trace_traceType=TRACE_Camera_Near;      //只寻近线
//                            //继续积分
//                            Encoder_Begin(ENCODER_MOTOR_2);
//                            Cross_status=CROSS_IN;
//                        }
//
//
//
//
//                            //----------------------------------------
//            }
//
//            else if(Cross_status==CROSS_IN){        //进入十字路口的弯道部分
//                                            //陀螺仪方式来判断是否到达最终阶段
//                                    if(1){}
//
//
//
//                                    //陀螺仪积分部分
//                                    if (Cross_measureType == GYROSCOPE_GYRO_X) {
//                                                                           if (fabs(Gyro_x) > Cross_angleEntry_Thre) {
//                                                                               //进入十字环岛
//                                                                               Cross_forceAngle_Status = 0;
//                                                                               Elec_pidStatus = 1;
//                                                                               Cross_status = CROSS_IN2;
//                                                                               Gyroscope_Clear(Cross_measureType);
//                                                                               Gyroscope_Begin(Cross_measureType);
//                                                                           }
//                                                                       }
//                                                                       else if (Cross_measureType == GYROSCOPE_GYRO_Y) {
//                                                                           if (fabs(Gyro_y) > Cross_angleEntry_Thre) {
//                                                                               //进入十字环岛
//                                                                               Cross_forceAngle_Status = 0;
//                                                                               Elec_pidStatus = 1;
//                                                                               Cross_status = CROSS_IN2;
//                                                                               Gyroscope_End(Cross_measureType);
//                                                                               Gyroscope_Clear(Cross_measureType);
//                                                                               Gyroscope_Begin(Cross_measureType);
//                                                                           }
//                                                                       }
//                                                                       else if (Cross_measureType == GYROSCOPE_GYRO_Z) {
//                                                                           if (fabs(Gyro_z) > Cross_angleEntry_Thre) {
//                                                                               //进入十字环岛
//                                                                               Cross_forceAngle_Status = 0;
//                                                                               Elec_pidStatus = 1;
//                                                                               Speed_set = 30;
//                                                                               Cross_status = CROSS_IN2;
//                                                                               Gyroscope_End(Cross_measureType);
//                                                                               Gyroscope_Clear(Cross_measureType);
//                                                                               Gyroscope_Begin(Cross_measureType);
//                                                                           }
//                                                                       }
//
//                                }
//
//
//
//
//
//
//
//                 else if (Cross_status==CROSS_IN2) {
//                             //开始搜索左线，跟着左线的中线走,同时最后用fabs(Gyro_)来判断是否到达cross_begin_2阶段
//
//
//
//                            if((Image_LptLeft_Found &&!Image_LptRight_Found)||(Image_LptLeft_Found&&Image_LptRight_Found)){          //左角点发现之后
//                                            Cross_status=CROSS_BEGIN2;
//                                            Encoder_Begin(ENCODER_MOTOR_2);
//                                       }
//                        }
//                 else if (Cross_status==CROSS_BEGIN2) {
//
//                     if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) ++none_left_line_cross;
//                                                 if (Image_rptsLeftsNum <0.2/ Image_sampleDist) ++none_right_line_cross;
//                                                     if (none_left_line_cross>2&&none_right_line_cross>2) {
//                                                         none_left_line_cross = 0;
//                                                         none_right_line_cross = 0;
//                                                         Cross_status = CROSS_RUNNING2;
//                                                         Trace_traceType = TRACE_Camera_Far;
//                                                         Gyroscope_Begin(Cross_measureType);     //开启陀螺仪
//                                                     }
//                                         }
//                 else if(Cross_status==CROSS_RUNNING2){
//                     if(Image_rptsLeftsNum > 0.2 / Image_sampleDist&&Image_rptsRightsNum > 0.2 / Image_sampleDist||abs(Encoder_sum_Motor1)>Cross_encoderRight_Thre){  //可以修改||
//                                                 Encoder_End(ENCODER_MOTOR_1);
//                                                 Encoder_Clear(ENCODER_MOTOR_1);
//                                                 Trace_traceType=TRACE_Camera_Near;
//                                                 Cross_status=CROSS_NONE;
//                                                 Trace_Status=TRACE_CENTERLINENEAR;
//                                             }
//                 }
//}
void handle_cross(){
    if(Cross_status == CROSS_BEGIN){
           PWMSetSteer(85.0);

           if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>EncoderCross_In_Thre&&fabs(Gyro_z)>Gyroscope_z_Cross_in_Thre){
                          Encoder_End(ENCODER_MOTOR_2);
                          Encoder_Clear(ENCODER_MOTOR_2);
                          Encoder_End(ENCODER_MOTOR_1);
                          Encoder_Clear(ENCODER_MOTOR_1);
                          Encoder_Begin(ENCODER_MOTOR_1);
                          Encoder_Begin(ENCODER_MOTOR_2);

                          Gyroscope_End(Cross_measureType);
                          Gyroscope_Clear(Cross_measureType);
                          Gyroscope_Begin(Cross_measureType);
                          Cross_status = CROSS_IN;
                      }
       }
    if(Cross_status == CROSS_IN){
        Trace_traceType=TRACE_Camera_Far;
        if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>EncoderCross_Thre){
                                      Encoder_End(ENCODER_MOTOR_2);
                                      Encoder_Clear(ENCODER_MOTOR_2);
                                      Encoder_End(ENCODER_MOTOR_1);
                                      Encoder_Clear(ENCODER_MOTOR_1);
                                      Encoder_Begin(ENCODER_MOTOR_1);
                                      Encoder_Begin(ENCODER_MOTOR_2);
                                      Cross_status = CROSS_RUNNING;
        }
    }
       if(Cross_status == CROSS_RUNNING){
           Trace_traceType=TRACE_Camera_Near;
           if(Image_rptsLeftNum>=Image_rptsRightNum){
           if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>3.2*EncoderCross_Thre&&Image_LptLeft_Found&&fabs(Gyro_z)>Gyroscope_z_Cross_running_Thre){
                   Encoder_End(ENCODER_MOTOR_2);
                   Encoder_Clear(ENCODER_MOTOR_2);
                   Encoder_End(ENCODER_MOTOR_1);
                   Encoder_Clear(ENCODER_MOTOR_1);
                   Encoder_Begin(ENCODER_MOTOR_1);
                   Encoder_Begin(ENCODER_MOTOR_2);

                   Gyroscope_End(Cross_measureType);
                   Gyroscope_Clear(Cross_measureType);
                   Cross_status = CROSS_IN2;
                  }
           }
           if(Image_rptsRightNum>=Image_rptsLeftNum){
                      if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>3.2*EncoderCross_Thre&&Image_LptRight_Found){
                              Encoder_End(ENCODER_MOTOR_2);
                              Encoder_Clear(ENCODER_MOTOR_2);
                              Encoder_End(ENCODER_MOTOR_1);
                              Encoder_Clear(ENCODER_MOTOR_1);
                              Encoder_Begin(ENCODER_MOTOR_1);
                              Encoder_Begin(ENCODER_MOTOR_2);
                              Cross_status = CROSS_IN2;
                             }
                      }
          }
       if(Cross_status == CROSS_IN2){
             if(Shift_Direction==SHIFT_RIGHT){
              PWMSetSteer(82.0);
             }
             if(Shift_Direction==SHIFT_LEFT){
              PWMSetSteer(92.0);
                          }
             if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>EncoderCross_IN2_Thre&&Shift_Direction==SHIFT_BOTH){
                     Encoder_End(ENCODER_MOTOR_2);
                     Encoder_Clear(ENCODER_MOTOR_2);
                     Encoder_End(ENCODER_MOTOR_1);
                     Encoder_Clear(ENCODER_MOTOR_1);
                     Encoder_Begin(ENCODER_MOTOR_1);
                     Encoder_Begin(ENCODER_MOTOR_2);
                     Cross_status = CROSS_END;
                    }
            }
       if(Cross_status == CROSS_END){   //循远线
           Trace_traceType=TRACE_Camera_Far;
          // Trace_traceType=TRACE_Camera_Far;
           if((abs(Encoder_sum_Motor2)+abs(Encoder_sum_Motor1))>EncoderCross_Thre&&Image_iptsLeftNum>10&&Image_iptsRightNum>10){           //重新找到左右线
                                 Cross_status = CROSS_NONE;
                                 Trace_traceType=TRACE_Camera_MID;
                                 Trace_Status=TRACE_CENTERLINENEAR;
                                 Encoder_End(ENCODER_MOTOR_2);
                                 Encoder_Clear(ENCODER_MOTOR_2);
                                 Encoder_End(ENCODER_MOTOR_1);
                                 Encoder_Clear(ENCODER_MOTOR_1);
                             }
       }
}


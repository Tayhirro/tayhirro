/*
 * xiao_circle.c
 *
 *  Created on: 2023年7月2日
 *      Author: Jayden_NaN
 */

#include "xiao_circle.h"
#include "xiao_encoder.h"
#include "xiao_image_processing.h"
#include "xiao_trace.h"
#include "xiao_gyroscope.h"
#include "xiao_pid.h"
//------------------------------------------------------------
//状态机
CIRCLE_STATUS Circle_status = CIRCLE_NONE;
//------------------------------------------------------------
//基本变量
int32 Circle_encoder;                       //编码器积分,防止出现一些重复触发等事件
int32 Circle_currentEncoder = 0;            //编码器当前的积分

//----------------------------------------
//环岛检测状态
uint8 Circle_leftBeginStatus_Camera = 0;    //摄像头判断左环岛开始状态机
uint8 Circle_rightBeginStatus_Camera = 0;   //摄像头判断右环岛开始状态机
uint8 Circle_leftBeginStatus_Elec = 0;      //电磁判断左环岛开始状态机
uint8 Circle_rightBeginStatus_Elec = 0;     //电磁判断右环岛开始状态机

//----------------------------------------
//摄像头 - 判断边线是否消失
uint8 none_left_line = 0,                   //判断左边线是否消失
        none_right_line = 0;                //判断右边线是否消失
uint8 have_left_line = 0,                   //判断是否存在左边线
        have_right_line = 0;                //判断是否存在右边线
//----------------------------------------
//电磁基本变量
uint8 Circle_ElecID[6] = {5, 2, 6,          //电感在Elec_data数据中的ID号
                        3, 4 , 1};
/*
 * 电感示意图:
 * |_1_|--------------------|_2_|--------------------|_3_|
 *                            |
 *                            |
 *                            |
 *                            |
 *                            |
 *                            |
 * |_4_|--------------------|_5_|--------------------|_6_|
 *
 * 注意:
 *      1. 均为横电感
 *      2. 电感数据通过上面的ID号进行映射
 *      3. left/right 仅仅指代左右环岛,与其它无关
 */

//------------------------------检测环岛部分------------------------------




//----------------------------------------
//电磁  -  环岛阈值处理
int16 Circle_middleElecDiff_Thre = 0;               //中间两颗电感的差值阈值
int16 Circle_middleElec_Front_Thre = 0;             //中间两颗电感前面电感的阈值
int16 Circle_middleElec_Back_Thre = 0;              //中间两颗电感后面电感的阈值
int16 Circle_leftElec_Front_Thre = 0;               //前左侧电感的阈值
int16 Circle_rightElec_Front_Thre = 0;              //前右侧电感的阈值
int16 Circle_leftElec_Back_Thre = 0;                //后左侧电感的阈值
int16 Circle_rightElec_Back_Thre = 0;               //后右侧电感的阈值

//----------------------------------------




//电磁  - 环岛大于阈值次数计数 (sc - satisfy condition)
uint8 Circle_elecLeft_scCnt = 0;                    //环岛判断电磁部分满足次数 - 左环岛
const uint8 Circle_elecLeft_scCnt_Thre = 0;         //环岛判断电磁满足次数阈值(次数大于阈值就判断预入环)
uint8 Circle_elecLeft_nscCnt = 0;                   //环岛判断电磁部分不满足次数
const uint8 Circle_elecLeft_nscCnt_Thre = 5;        //环岛判断电磁不满足次数阈值(次数小于阈值就判断非环岛)
uint8 Circle_elecRight_scCnt = 0;                   //环岛判断电磁部分满足次数 - 右环岛
const uint8 Circle_elecRight_scCnt_Thre = 0;        //环岛判断电磁满足次数阈值(次数大于阈值就判断预入环)
uint8 Circle_elecRight_nscCnt = 0;                  //环岛判断电磁部分不满足次数
const uint8 Circle_elecRight_nscCnt_Thre = 5;       //环岛判断电磁不满足次数阈值(次数小于阈值就判断非环岛)





//----------------------------------------
//摄像头 - 环岛大于阈值次数计数 (sc - satisfy condition)
uint8 Circle_cameraLeft_scCnt = 0;                  //环岛判断摄像头部分满足次数
const uint8 Circle_cameraLeft_scCnt_Thre = 0;       //环岛判断摄像头满足次数阈值(次数大于阈值就判断预入环)
uint8 Circle_cameraLeft_nscCnt = 0;                 //环岛判断摄像头部分不满足次数
const uint8 Circle_cameraLeft_nscCnt_Thre = 0;      //环岛判断摄像头部分不满足次数阈值(次数大于阈值就判断非环岛)
uint8 Circle_cameraRight_scCnt = 0;                 //环岛判断摄像头部分满足次数
const uint8 Circle_cameraRight_scCnt_Thre = 0;      //环岛判断摄像头满足次数阈值(次数大于阈值就判断预入环)
uint8 Circle_cameraRight_nscCnt = 0;                //环岛判断摄像头部分不满足次数
const uint8 Circle_cameraRight_nscCnt_Thre = 0;     //环岛判断摄像头部分不满足次数阈值(次数大于阈值就判断非环岛)

//------------------------------预入环部分------------------------------
int16 Circle_encoderLeft = 0;                       //左轮编码器积分
int16 Circle_encoderRight = 0;                      //右轮编码器积分
//------------------------------补充------------------------------
//摄像头判断时候,编码器积分阈值在16000~19000之间
int16 Circle_encoderLeft_Thre = 15500;              //左轮编码器积分阈值
int16 Circle_encoderRight_Thre = 15500;             //右轮编码器积分阈值
int8 Circle_speedVarLeft_Motor1 = 13;               //强制打角的时候,电机1的变化量   -   左环岛
int8 Circle_speedVarLeft_Motor2 = -13;              //强制打角的时候,电机2的变化量   -   左环岛
int8 Circle_speedVarRight_Motor1 = -13;             //强制打角的时候,电机1的变化量   -   右环岛
int8 Circle_speedVarRight_Motor2 = 13;              //强制打角的时候,电机2的变化量   -   右环岛
uint8 Circle_forceAngleLeft_Status = 0;             //左环岛强制打角状态机
uint8 Circle_forceAngleRight_Status = 0;            //右环岛强制打角状态机
//--------------------二次判断--------------------
int16 Circle_encoderLeft_Ther_2ndJd = 16000;        //编码器二次判断阈值           -    左环岛
int16 Circle_encoderRight_Ther_2ndJd = 16000;       //编码器二次判断阈值           -    右环岛
uint8 Circle_encoderLeft_isCircle_2ndJd = 0;        //是否是环岛状态位            -    左环岛
uint8 Circle_encoderRight_isCircle_2ndJd = 0;       //是否是环岛状态位            -    右环岛
int16 Circle_Elec_Thre_LeftFront = 3000;            //二次检测左上电感阈值
int16 Circle_Elec_Thre_RightFront = 3000;           //二次检测右上电感阈值

uint8 Circle_isCircle_LeftSC = 0;                   //左环岛满足条件计数
uint8 Circle_isCircle_LeftSC_Thre = 5;              //左环岛满足条件次数阈值
uint8 Circle_isCircle_LeftNSC = 0;                  //左环岛不满足条件计数
uint8 Circle_isCircle_LeftNSC_Thre = 5;             //左环岛不满足条件次数阈值

uint8 Circle_isCircle_RightSC = 0;                   //左环岛满足条件计数
uint8 Circle_isCircle_RightSC_Thre = 5;              //左环岛满足条件次数阈值
uint8 Circle_isCircle_RightNSC = 0;                  //左环岛不满足条件计数
uint8 Circle_isCircle_RightNSC_Thre = 5;             //左环岛不满足条件次数阈值


//------------------------------入环部分------------------------------
float Circle_angleLeftEntry_Thre = 45.0;             //左环岛入环角度积分阈值
float Circle_angleRightEntry_Thre = 45.0;            //右环岛入环角度积分阈值


//------------------------------环内部分------------------------------

//------------------------------预出环部分------------------------------
float Circle_circleAngle_Thre = 280.0;              //出环角度积分阈值(单位为度)
uint8 Circle_circleAngle_scCnt = 0;                 //角度积分大于阈值次数计数
uint8 Circle_circleAngle_scCnt_Ther = 3;            //角度积分大于阈值次数计数阈值
uint8 Circle_circleAngle_nscCnt = 0;                //角度积分不满足条件计数
uint8 Circle_circleAngel_nscCnt_Thre = 3;           //角度积分不满足条件计数阈值

//------------------------------出环部分------------------------------
int8 Circle_speedVarLeftOut_Motor1 = 8;               //强制打角的时候,电机1的变化量   -   左环岛
int8 Circle_speedVarLeftOut_Motor2 = -8;              //强制打角的时候,电机2的变化量   -   左环岛
int8 Circle_speedVarRightOut_Motor1 = -8;             //强制打角的时候,电机1的变化量   -   右环岛
int8 Circle_speedVarRightOut_Motor2 = 8;              //强制打角的时候,电机2的变化量   -   右环岛
float Circle_angleLeftExit_Thre = 65.0;              //左环岛出环角度积分阈值
float Circle_angleRightExit_Thre = 65.0;             //右环岛出环角度积分阈值

//------------------------------结束------------------------------
int16 Circle_encoderExitLeft_Thre = 5000;              //出左环编码器积分阈值
int16 Circle_encoderExitRight_Thre = 5000;             //出右环编码器积分阈值

//------------------------------多环岛部分------------------------------
uint8 Circle_multiCircle_Status = 0;
//------------------------------出环岛加速------------------------------
uint8 Circle_speedAcc_Status = 0;
int16 EncoderCircle_In_Thre =2000;
int16 EncoderCircle_Running_Thre =1500;
int16 EncoderCircle_Out_Thre=6400;
int16 EncoderCircle_End_Thre=4000;
int16 EncoderCircle_Pre_Thre=4000;
//陀螺仪测量角度的时候使用的变量 - (假定先使用x轴陀螺仪)
GYROSCOPE_MEASURE_TYPE Circle_measureType = GYROSCOPE_GYRO_Z;
float Gyroscope_z_Circle_running_Thre=360.0;

void Circle_CheckCamera(void) {
    //左环岛
    if (Circle_status == CIRCLE_NONE && Image_LptLeft_Found && !Image_LptRight_Found && Image_isStraightRight&&Trace_Status==TRACE_CENTERLINENEAR) {
        //测试代码
        Trace_Status=TRACE_CIRCLE_LEFT;
        Circle_status = CIRCLE_LEFT_BEGIN;
        Trace_traceType = TRACE_Camera_Near_RIGHT;
        Encoder_Begin(ENCODER_MOTOR_2);
        Gyroscope_Begin(Circle_measureType);
//        put_int32(70, 1);
        gpio_set_level(P20_8, GPIO_LOW);
    }
    //右环岛
    if (Circle_status == CIRCLE_NONE && Image_LptRight_Found && !Image_LptLeft_Found && Image_isStraightLeft) {
        Trace_Status=TRACE_CIRCLE_RIGHT;
        Circle_status = CIRCLE_RIGHT_BEGIN;
        Trace_traceType = TRACE_Camera_Near_LEFT;
        Encoder_Begin(ENCODER_MOTOR_1);
//        put_int32(71, 1);
        gpio_set_level(P20_9, GPIO_LOW);
    }
}
void handle_circle_left(){
    //------------------------------处理左环岛------------------------------
    //------------------------------
    //1. 开始,寻右边线
    //2. 入环,寻内圆左线
    //3. 在环内,正常巡线(直接用电磁 - 后者用摄像头寻外圆右线)
    //4. 出环,寻内圆
    //5. 走出圆环,寻右线(或者用电磁跑)
    //------------------------------
    //开始
    if (Circle_status == CIRCLE_LEFT_BEGIN) {
        Trace_traceType = TRACE_Camera_Near_RIGHT;           //近处寻右线

        //先丢左线后右线
        if (Image_rptsLeftNum < 0.2 / Image_sampleDist) {++none_left_line;}
        if (Image_rptsLeftNum > 0.5 / Image_sampleDist && none_left_line > 2) {
            ++have_left_line;
            if (have_left_line >= 1&&abs(Encoder_sum_Motor2)>EncoderCircle_In_Thre) {
                Circle_status = CIRCLE_LEFT_IN_PRE;
                none_left_line = 0;
                have_left_line = 0;
                Encoder_End(ENCODER_MOTOR_2);
                Encoder_Clear(ENCODER_MOTOR_2);
                Encoder_Begin(ENCODER_MOTOR_2);
                Trace_traceType = TRACE_Camera_Near_LEFT;
            }
        }
    }
    else if(Circle_status == CIRCLE_LEFT_IN_PRE){
        PWMSetSteer(98.0);
        if(Image_rptsRightNum==0&&abs(Encoder_sum_Motor2)>EncoderCircle_Pre_Thre){
                        Circle_status=CIRCLE_LEFT_IN;
                        Encoder_End(ENCODER_MOTOR_2);
                        Encoder_Clear(ENCODER_MOTOR_2);
                        Encoder_Begin(ENCODER_MOTOR_2);
             //           Trace_traceType = TRACE_Camera_Near_LEFT;
        }
    }
    //入环,寻内圆左线
    else if (Circle_status == CIRCLE_LEFT_IN) {
        //Trace_traceType = TRACE_Camera_Near_LEFT;
        //检测到右边线时切换到下一个状态
        PWMSetSteer(100.0);
        if(Image_rptsRightNum!=0&&abs(Encoder_sum_Motor2)*2>EncoderCircle_Running_Thre){
            Circle_status=CIRCLE_LEFT_RUNNING;
            Encoder_End(ENCODER_MOTOR_2);
            Encoder_Clear(ENCODER_MOTOR_2);
            Encoder_Begin(ENCODER_MOTOR_2);
            Trace_traceType = TRACE_Camera_Near_RIGHT;
        }
    }
    //在环内,正常巡线 (先做摄像头巡线)
    else if (Circle_status == CIRCLE_LEFT_RUNNING) {
        Trace_traceType = TRACE_Camera_Near_RIGHT;

        //当找到右L角点的时候
        if (Image_LptRight_Found) Image_rptsRightsNum = Image_rptsRightcNum = Image_LptRight_rptsRights_id;
        //外环拐点
        if (Image_LptRight_Found && Image_LptRight_rptsRights_id < 0.4 / Image_sampleDist&&abs(Encoder_sum_Motor2)*2>EncoderCircle_Out_Thre){
            Circle_status = CIRCLE_LEFT_OUT;
                        Encoder_End(ENCODER_MOTOR_2);
                        Encoder_Clear(ENCODER_MOTOR_2);
                        Encoder_Begin(ENCODER_MOTOR_2);
                        Trace_traceType = TRACE_Camera_Near_RIGHT;
        }
    }
    //出环
    else if (Circle_status == CIRCLE_LEFT_OUT) {
        Trace_traceType = TRACE_Camera_Near_RIGHT;
        PWMSetSteer(100.0);
        if (Image_rptsRightNum>5&&abs(Encoder_sum_Motor2)>EncoderCircle_End_Thre&&fabs(Gyro_z)>Gyroscope_z_Circle_running_Thre) {
            Circle_status = CIRCLE_LEFT_END;
            Encoder_End(ENCODER_MOTOR_2);
            Encoder_Clear(ENCODER_MOTOR_2);
            Encoder_Begin(ENCODER_MOTOR_2);
            Gyroscope_End(Circle_measureType);
            Gyroscope_Clear(Circle_measureType);
            Trace_traceType = TRACE_Camera_Far_RIGHT;
        }
    }
    else if (Circle_status == CIRCLE_LEFT_END) {
        Trace_traceType = TRACE_Camera_Far_RIGHT;
        if(abs(Encoder_sum_Motor2)>1.5*EncoderCircle_End_Thre)
        Trace_traceType=TRACE_Camera_MID;
        Trace_Status=TRACE_CENTERLINENEAR;
        Circle_status = CIRCLE_NONE;
    }

}
void handle_circle_right(){
    //------------------------------处理左环岛------------------------------
    //------------------------------
    //1. 开始,寻右边线
    //2. 入环,寻内圆左线
    //3. 在环内,正常巡线(直接用电磁 - 后者用摄像头寻外圆右线)
    //4. 出环,寻内圆
    //5. 走出圆环,寻右线(或者用电磁跑)
    //------------------------------
    //开始
    if (Circle_status == CIRCLE_RIGHT_BEGIN) {
        Trace_traceType = TRACE_Camera_Near_LEFT;           //近处寻右线

        //先丢右线后左线
        if (Image_rptsRightNum < 0.2 / Image_sampleDist) {++none_right_line;}
        if (Image_rptsRightNum > 0.5 / Image_sampleDist && none_right_line > 2) {
            ++have_right_line;
            if (have_right_line >= 1&&abs(Encoder_sum_Motor1)>EncoderCircle_In_Thre) {
                Circle_status = CIRCLE_RIGHT_IN_PRE;
                none_right_line = 0;
                have_right_line = 0;
                Encoder_End(ENCODER_MOTOR_1);
                Encoder_Clear(ENCODER_MOTOR_1);
                Encoder_Begin(ENCODER_MOTOR_1);
                Trace_traceType = TRACE_Camera_Near_RIGHT;
            }
        }
    }
    else if(Circle_status == CIRCLE_RIGHT_IN_PRE){
        PWMSetSteer(85.0);
        if(Image_rptsLeftNum==0&&abs(Encoder_sum_Motor1)*0.6>EncoderCircle_Pre_Thre){
                        Circle_status=CIRCLE_RIGHT_IN;
                        Encoder_End(ENCODER_MOTOR_1);
                        Encoder_Clear(ENCODER_MOTOR_1);
                        Encoder_Begin(ENCODER_MOTOR_1);
             //           Trace_traceType = TRACE_Camera_Near_LEFT;
        }
    }
    //入环,寻内圆左线
    else if (Circle_status == CIRCLE_RIGHT_IN) {
        //Trace_traceType = TRACE_Camera_Near_LEFT;
        //检测到右边线时切换到下一个状态
        PWMSetSteer(80.0);
        if(Image_rptsLeftNum!=0&&abs(Encoder_sum_Motor1)*2>EncoderCircle_Running_Thre){
            Circle_status=CIRCLE_RIGHT_RUNNING;
            Encoder_End(ENCODER_MOTOR_1);
            Encoder_Clear(ENCODER_MOTOR_1);
            Encoder_Begin(ENCODER_MOTOR_1);
            Trace_traceType = TRACE_Camera_Near_LEFT;
        }
    }
    //在环内,正常巡线 (先做摄像头巡线)
    else if (Circle_status == CIRCLE_RIGHT_RUNNING) {
        Trace_traceType = TRACE_Camera_Near_LEFT;

        //当找到左L角点的时候
        if (Image_LptLeft_Found) Image_rptsLeftsNum = Image_rptsLeftcNum = Image_LptLeft_rptsLefts_id;
        //外环拐点
        if (Image_LptLeft_Found && Image_LptLeft_rptsLefts_id < 0.4 / Image_sampleDist&&abs(Encoder_sum_Motor1)*2>EncoderCircle_Out_Thre){
            Circle_status = CIRCLE_RIGHT_OUT;
                        Encoder_End(ENCODER_MOTOR_1);
                        Encoder_Clear(ENCODER_MOTOR_1);
                        Encoder_Begin(ENCODER_MOTOR_1);
                        Trace_traceType = TRACE_Camera_Near_LEFT;
        }
    }
    //出环
    else if (Circle_status == CIRCLE_RIGHT_OUT) {
        Trace_traceType = TRACE_Camera_Near_LEFT;
        PWMSetSteer(80.0);
        if (Image_rptsLeftNum>5&&abs(Encoder_sum_Motor1)>EncoderCircle_End_Thre&&fabs(Gyro_z)>Gyroscope_z_Circle_running_Thre) {
            Circle_status = CIRCLE_RIGHT_END;
            Encoder_End(ENCODER_MOTOR_1);
            Encoder_Clear(ENCODER_MOTOR_1);
            Encoder_Begin(ENCODER_MOTOR_1);
            Gyroscope_End(Circle_measureType);
            Gyroscope_Clear(Circle_measureType);
            Trace_traceType = TRACE_Camera_Far_LEFT;
        }
    }
    else if (Circle_status == CIRCLE_RIGHT_END) {
        Trace_traceType = TRACE_Camera_Far_LEFT;
        if(
                abs(Encoder_sum_Motor1)>1.5*EncoderCircle_End_Thre)
        Trace_traceType=TRACE_Camera_MID;
        Trace_Status=TRACE_CENTERLINENEAR;
        Circle_status = CIRCLE_NONE;
    }

}

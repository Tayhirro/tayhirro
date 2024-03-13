/*
 * xiao_circle.c
 *
 *  Created on: 2023年7月2日
 *      Author: Jayden_NaN
 */

#include "xiao_circle.h"
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


//陀螺仪测量角度的时候使用的变量 - (假定先使用x轴陀螺仪)
GYROSCOPE_MEASURE_TYPE Circle_measureType = GYROSCOPE_GYRO_X;

/*
 * @brief               使用图传发送状态情况
 * @attention           需要图传进行初始化,否则无法使用
 */
void Circle_PutStatus(void) {
    put_int32(72, Circle_status);
}

/*
 * @brief               通过检测角点的值,判断是左环岛还是右环岛
 * @attention           使用摄像头检测,需要对摄像头进行初始化
 */
void Circle_CheckCamera(void) {
    //左环岛
    if (Circle_status == CIRCLE_NONE && Image_LptLeft_Found && !Image_LptRight_Found && Image_isStraightRight) {
        //测试代码
        Circle_status = CIRCLE_LEFT_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_2);
//        put_int32(70, 1);
        gpio_set_level(P20_8, GPIO_LOW);
    }
    //右环岛
    if (Circle_status == CIRCLE_NONE && Image_LptRight_Found && !Image_LptLeft_Found && Image_isStraightLeft) {
        Circle_status = CIRCLE_RIGHT_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_1);
//        put_int32(71, 1);
        gpio_set_level(P20_9, GPIO_LOW);
    }
}

/*
 * @brief               电磁检测是否进入环岛
 * @attention           目前使用最边缘两侧横电感和中间一个电感进行环岛的判断
 *                      使用两条电感进行处理
 */
static void Circle_CheckElec() {
    //------------------------------
    //左环岛处理
//    if (Elec_data[Circle_ElecID[1]] > Circle_middleElec_Front_Thre &&
//            (Elec_data[Circle_ElecID[1]] - Elec_data[Circle_ElecID[3]]) > Circle_middleElecDiff_Thre &&
//            Elec_data[Circle_ElecID[0]] > Circle_leftElec_Front_Thre &&
//            Circle_leftBeginStatus_Elec == 0 &&
//            Circle_status == CIRCLE_NONE) {
//        Circle_elecLeft_nscCnt = 0;
//        ++Circle_elecLeft_scCnt;
//        if (Circle_elecLeft_scCnt > Circle_elecLeft_scCnt_Thre)
//            Circle_leftBeginStatus_Elec = 1;
//    }
//    else {
//        if (Circle_elecLeft_scCnt != 0) {
//            ++Circle_elecLeft_nscCnt;
//            if (Circle_elecLeft_nscCnt > Circle_elecLeft_nscCnt_Thre) {
//                Circle_elecLeft_scCnt = 0;
//                Circle_elecLeft_nscCnt = 0;
//            }
//        }
//    }
    //右环岛处理
//    if (Elec_data[Circle_ElecID[1]] > Circle_middleElec_Front_Thre &&
//            (Elec_data[Circle_ElecID[1]] - Elec_data[Circle_ElecID[3]]) > Circle_middleElecDiff_Thre &&
//            Elec_data[Circle_ElecID[2]] > Circle_rightElec_Front_Thre &&
//            Circle_rightBeginStatus_Elec == 0 &&
//            Circle_status == CIRCLE_NONE) {
//        Circle_elecRight_nscCnt = 0;
//        ++Circle_elecRight_scCnt;
//        if (Circle_elecRight_scCnt > Circle_elecRight_scCnt_Thre)
//            Circle_rightBeginStatus_Elec = 1;
//    }
//    else {
//        if (Circle_elecRight_scCnt != 0) {
//            ++Circle_elecRight_nscCnt;
//            if (Circle_elecRight_nscCnt > Circle_elecRight_nscCnt_Thre) {
//                Circle_elecRight_scCnt = 0;
//                Circle_elecRight_nscCnt = 0;
//            }
//        }
//    }
}


static void Circle_Begin(CIRCLE_CHECK_METHOD checkMethod) {
    if (Circle_status == CIRCLE_NONE) {
        //单纯使用电磁判断
        if (checkMethod == CIRCLE_CHECK_ELEC) {
            if (Circle_leftBeginStatus_Elec == 1) {
                Circle_status = CIRCLE_LEFT_BEGIN;
                Circle_leftBeginStatus_Elec = 0;
            }
            if (Circle_rightBeginStatus_Elec == 1) {
                Circle_status = CIRCLE_RIGHT_BEGIN;
                Circle_rightBeginStatus_Elec = 0;
            }
        }
        //单纯使用摄像头判断
        else if (checkMethod == CIRCLE_CHECK_CAMERA) {
            if (Circle_leftBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_LEFT_BEGIN;
                Circle_leftBeginStatus_Camera = 0;
            }
            if (Circle_rightBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_RIGHT_BEGIN;
                Circle_rightBeginStatus_Camera = 0;
            }
        }
        //摄像头和电磁同时使用
        else if (checkMethod == CIRCLE_CHECK_BOTH) {
            if (Circle_leftBeginStatus_Elec == 1 && Circle_leftBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_LEFT_BEGIN;
                Circle_leftBeginStatus_Camera = 0;
                Circle_leftBeginStatus_Elec = 0;
            }
            if (Circle_rightBeginStatus_Elec == 1 && Circle_rightBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_RIGHT_BEGIN;
                Circle_rightBeginStatus_Camera = 0;
                Circle_rightBeginStatus_Elec = 0;
            }
        }
    }
}


/*
 * @brief               在环岛进行寻迹(使用电磁)
 * @example
 */
void Circle_RunElec(CIRCLE_PRE_ENTRY_METHOD entryMethod) {
    //------------------------------处理左环岛------------------------------
    //----------------------------------------
    //1. 找到到达中点的位置
    //      1. 使用编码器积分
    //      2. 使用电磁判断
    //      3. 使用摄像头判断          - 预入环
    //2. 使用差速强制打角度 + 陀螺仪积分  - 入环
    //3. 正常巡线                     - 在环内
    //4. 使用电感判断阈值               - 预出环
    //      优先使用角度计算
    //5. 使用差速强制打角度 + 阈值判断    - 出环
    if (Circle_status == CIRCLE_LEFT_BEGIN) {
        //------------------------------
        //1.预入环使用编码器进行积分
        if (entryMethod == CIRCLE_ENTRY_ENCODER) {

/*
            //二次检测阶段
            if (abs(Encoder_sum_Motor2) > Circle_encoderLeft_Ther_2ndJd && Circle_encoderLeft_isCircle_2ndJd == 0) {
                if (Elec_data[Circle_ElecID[0]] > Circle_Elec_Thre_LeftFront && Elec_data[Circle_ElecID[2]] > Circle_Elec_Thre_RightFront) {
                    Circle_elecLeft_nscCnt = 0;
                    ++Circle_elecLeft_scCnt;
                    if (Circle_elecLeft_scCnt > Circle_elecLeft_scCnt_Thre) {
                        Circle_elecLeft_scCnt = 0;
                        Circle_encoderLeft_isCircle_2ndJd = 1;
                    }
                }
                else if (Circle_encoderLeft_isCircle_2ndJd == 0 && (Elec_data[Circle_ElecID[0]] < Circle_Elec_Thre_LeftFront || Elec_data[Circle_ElecID[2]] < Circle_Elec_Thre_RightFront) && Circle_elecLeft_scCnt != 0) {
                    ++Circle_elecLeft_nscCnt;
                    if (Circle_elecLeft_nscCnt > Circle_elecLeft_nscCnt_Thre) {
                        Circle_elecLeft_scCnt = 0;
                        Circle_elecLeft_nscCnt = 0;
                        Circle_status = CIRCLE_NONE;
                        gpio_set_level(P20_8, GPIO_HIGH);
                    }
                }
            }
*/
            //----------------------------------------
            //强制打角阶段
            //满足条件,进入强制打角,编码器积分关掉(同时清0),角度积分打开
            if (abs(Encoder_sum_Motor2) > Circle_encoderLeft_Thre && Circle_forceAngleLeft_Status == 0) {
               // Beep_Tweet();
                Elec_pidStatus = 0;
                Motor_1Target = (int16)Speed_set + Circle_speedVarLeft_Motor1;
                Motor_2Target = (int16)Speed_set + Circle_speedVarLeft_Motor2;
                Encoder_End(ENCODER_MOTOR_2);
                Encoder_Clear(ENCODER_MOTOR_2);
                Gyroscope_Begin(Circle_measureType);
                Circle_forceAngleLeft_Status = 1;
                Circle_status = CIRCLE_LEFT_IN;
            }
        }
        //2.预入环使用电感阈值进行判断
       // else if (entryMethod == CIRCLE_ENTRY_ELEC_THRE) {

       // }
        //3.预入环使用摄像头进行判断
        else if (entryMethod == CIRCLE_ENTRY_CAMERA) {

        }
    }

    //3. 正常巡线                     - 在环内
    else if (Circle_status == CIRCLE_LEFT_IN) {
        if (Circle_forceAngleLeft_Status == 1) {

            if (Circle_measureType == GYROSCOPE_GYRO_X) {
                if (fabs(Gyro_x) > Circle_angleLeftEntry_Thre) {
                    //进入环岛
                    Circle_forceAngleLeft_Status = 0;
                    Elec_pidStatus = 1;
                    Circle_status = CIRCLE_LEFT_RUNNING;
                }
            }
            else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
                if (fabs(Gyro_y) > Circle_angleLeftEntry_Thre) {
                    //进入环岛
                    Circle_forceAngleLeft_Status = 0;
                    Elec_pidStatus = 1;
                    Circle_status = CIRCLE_LEFT_RUNNING;
                }
            }
            else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
                if (fabs(Gyro_z) > Circle_angleLeftEntry_Thre) {
                    //进入环岛
                    Circle_forceAngleLeft_Status = 0;
                    Elec_pidStatus = 1;
                    Speed_set = 30;
                    Circle_status = CIRCLE_LEFT_RUNNING;
                }
            }
        }
    }
    else if (Circle_status == CIRCLE_LEFT_RUNNING) {
        //预出环判断
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            if (fabs(Gyro_x) > Circle_circleAngle_Thre) {
                //准备出环岛
                Elec_pidStatus = 0;
//                ++Circle_circleAngle_scCnt;
//                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
//                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_LEFT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
//                }
//                else {
//                    if (Circle_circleAngle_scCnt != 0) {
//                        ++Circle_circleAngle_nscCnt;
//                        //这种情况就说明是误判
//                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
//                            Circle_circleAngle_nscCnt = 0;
//                            Circle_circleAngle_scCnt = 0;
//                        }
//                    }
//                }
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
            if (Gyro_y > Circle_angleLeftEntry_Thre) {
                //准备出环岛
                ++Circle_circleAngle_scCnt;
                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_LEFT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
                }
                else {
                    if (Circle_circleAngle_scCnt != 0) {
                        ++Circle_circleAngle_nscCnt;
                        //这种情况就说明是误判
                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
                            Circle_circleAngle_nscCnt = 0;
                            Circle_circleAngle_scCnt = 0;
                        }
                    }
                }
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (Gyro_z > Circle_angleLeftEntry_Thre) {
                //准备出环岛
                ++Circle_circleAngle_scCnt;
                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_LEFT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarLeftOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
                }
                else {
                    if (Circle_circleAngle_scCnt != 0) {
                        ++Circle_circleAngle_nscCnt;
                        //这种情况就说明是误判
                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
                            Circle_circleAngle_nscCnt = 0;
                            Circle_circleAngle_scCnt = 0;
                        }
                    }
                }
            }
        }
    }
    else if (Circle_status == CIRCLE_LEFT_OUT) {
        //预出环状态
        //电磁pid关闭,强制打角
        //大于状态以后,强制走直线,然后用编码器进行积分
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            if (fabs(Gyro_x) > Circle_angleLeftExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_2);
                Circle_status = CIRCLE_LEFT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
            if (fabs(Gyro_y) > Circle_angleLeftExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_2);
                Circle_status = CIRCLE_LEFT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (fabs(Gyro_z) > Circle_angleLeftExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_2);
                Circle_status = CIRCLE_LEFT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
    }
    else if (Circle_status == CIRCLE_LEFT_END) {
        if (Encoder_sum_Motor2 > Circle_encoderExitLeft_Thre) {
            Elec_pidStatus = 1;

            if (Circle_multiCircle_Status == 0) {
                Circle_status = CIRCLE_ONLY_ONE;
                if (Circle_speedAcc_Status == 1) {
                    Speed_set += Speed_accVar;
                }
            }
            else {
                Circle_status = CIRCLE_NONE;
            }

            Encoder_End(ENCODER_MOTOR_2);
        }
    }
    //------------------------------处理右环岛------------------------------
    //----------------------------------------
    //1. 找到到达中点的位置
    //      1. 使用编码器积分
    //      2. 使用电磁判断
    //      3. 使用摄像头判断          - 预入环
    //2. 使用差速强制打角度 + 陀螺仪积分  - 入环
    //3. 正常巡线                     - 在环内
    //4. 使用电感判断阈值               - 预出环
    //5. 使用差速强制打角度 + 阈值判断    - 出环
    else if (Circle_status == CIRCLE_RIGHT_BEGIN) {
        //------------------------------
        //1.预入环使用编码器进行积分
        if (entryMethod == CIRCLE_ENTRY_ENCODER) {
            //为了防止电感影响差速抖动,这里直接将Elec_pidStatus设置为0
            //Elec_pidStatus = 0;

            //二次检测阶段
/*
            if (abs(Encoder_sum_Motor1) > Circle_encoderRight_Ther_2ndJd && Circle_encoderRight_isCircle_2ndJd == 0) {
                if (Elec_data[Circle_ElecID[0]] > Circle_Elec_Thre_LeftFront && Elec_data[Circle_ElecID[2]] > Circle_Elec_Thre_RightFront) {
                    Circle_elecRight_nscCnt = 0;
                    ++Circle_elecRight_scCnt;
                    if (Circle_elecRight_scCnt > Circle_elecRight_scCnt_Thre) {
                        Circle_elecRight_scCnt = 0;
                        Circle_encoderRight_isCircle_2ndJd = 1;
                    }
                }
                else if ((Elec_data[Circle_ElecID[0]] < Circle_Elec_Thre_LeftFront || Elec_data[Circle_ElecID[2]] < Circle_Elec_Thre_RightFront) && Circle_elecLeft_scCnt != 0) {
                    ++Circle_elecRight_nscCnt;
                    if (Circle_elecRight_nscCnt > Circle_elecRight_scCnt_Thre) {
                        Circle_elecRight_scCnt = 0;
                        Circle_elecRight_nscCnt = 0;
                        Circle_status = CIRCLE_NONE;
                        gpio_set_level(P20_9, GPIO_HIGH);
                    }
                }
            }
*/

            //----------------------------------------
            //强制打角阶段
            //满足条件,进入强制打角,编码器积分关掉(同时清0),角度积分打开
            if (abs(Encoder_sum_Motor1) > Circle_encoderRight_Thre && Circle_forceAngleRight_Status == 0) {
                Elec_pidStatus = 0;
                Motor_1Target = (int16)Speed_set + Circle_speedVarRight_Motor1;
                Motor_2Target = (int16)Speed_set + Circle_speedVarRight_Motor2;
                Encoder_End(ENCODER_MOTOR_1);
                Encoder_Clear(ENCODER_MOTOR_1);
                Gyroscope_Begin(Circle_measureType);
                Circle_forceAngleRight_Status = 1;
                Circle_status = CIRCLE_RIGHT_IN;
            }
        }
        //2.预入环使用电感阈值进行判断
        else if (entryMethod == CIRCLE_ENTRY_ELEC_THRE) {

        }
        //3.预入环使用摄像头进行判断
        else if (entryMethod == CIRCLE_ENTRY_CAMERA) {

        }
    }
    else if (Circle_status == CIRCLE_RIGHT_IN) {
        if (Circle_forceAngleRight_Status == 1) {
            if (Circle_measureType == GYROSCOPE_GYRO_X) {
                if (fabs(Gyro_x) > Circle_angleRightEntry_Thre) {
                    //进入环岛
                    Circle_forceAngleRight_Status = 0;
                    Elec_pidStatus = 1;
                    Circle_status = CIRCLE_RIGHT_RUNNING;
                }
            }
            else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
                if (fabs(Gyro_y) > Circle_angleRightEntry_Thre) {
                    //进入环岛
                    Circle_forceAngleRight_Status = 0;
                    Elec_pidStatus = 1;
                    Circle_status = CIRCLE_RIGHT_RUNNING;
                }
            }
            else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
                if (fabs(Gyro_z) > Circle_angleRightEntry_Thre) {
                    //进入环岛
                    Circle_forceAngleRight_Status = 0;
                    Elec_pidStatus = 1;
                    Circle_status = CIRCLE_RIGHT_RUNNING;
                }
            }
        }
    }
    else if (Circle_status == CIRCLE_RIGHT_RUNNING) {
        //预出环判断
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            if (fabs(Gyro_x) > Circle_circleAngle_Thre) {
                //准备出环岛
//                ++Circle_circleAngle_scCnt;
//                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_RIGHT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarRightOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarRightOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
//                }
//                else {
//                    if (Circle_circleAngle_scCnt != 0) {
//                        ++Circle_circleAngle_nscCnt;
//                        //这种情况就说明是误判
//                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
//                            Circle_circleAngle_nscCnt = 0;
//                            Circle_circleAngle_scCnt = 0;
//                        }
//                    }
//                }
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
            if (fabs(Gyro_y) > Circle_angleRightEntry_Thre) {
                //准备出环岛
                ++Circle_circleAngle_scCnt;
                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_RIGHT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarRightOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarRightOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
                }
                else {
                    if (Circle_circleAngle_scCnt != 0) {
                        ++Circle_circleAngle_nscCnt;
                        //这种情况就说明是误判
                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
                            Circle_circleAngle_nscCnt = 0;
                            Circle_circleAngle_scCnt = 0;
                        }
                    }
                }
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (fabs(Gyro_z) > Circle_angleRightEntry_Thre) {
                //准备出环岛
                ++Circle_circleAngle_scCnt;
                if (Circle_circleAngle_scCnt > Circle_circleAngle_scCnt_Ther) {
                    Circle_circleAngle_scCnt = 0;
                    Circle_status = CIRCLE_RIGHT_OUT;
                    Elec_pidStatus = 0;
                    Motor_1Target = (int16)Speed_set + Circle_speedVarRightOut_Motor1;
                    Motor_2Target = (int16)Speed_set + Circle_speedVarRightOut_Motor2;
                    Gyroscope_Clear(Circle_measureType);
                }
                else {
                    if (Circle_circleAngle_scCnt != 0) {
                        ++Circle_circleAngle_nscCnt;
                        //这种情况就说明是误判
                        if (Circle_circleAngle_nscCnt > Circle_circleAngel_nscCnt_Thre) {
                            Circle_circleAngle_nscCnt = 0;
                            Circle_circleAngle_scCnt = 0;
                        }
                    }
                }
            }
        }
    }
    else if (Circle_status == CIRCLE_RIGHT_OUT) {
        //预出环状态
        //电磁pid关闭,强制打角
        //大于状态以后,强制走直线,然后用编码器进行积分
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            if (fabs(Gyro_x) > Circle_angleRightExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_1);
                Circle_status = CIRCLE_RIGHT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (fabs(Circle_measureType) == GYROSCOPE_GYRO_Y) {
            if (Gyro_y > Circle_angleRightExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_1);
                Circle_status = CIRCLE_RIGHT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (fabs(Gyro_z) > Circle_angleRightExit_Thre) {
                Motor_1Target = (int16)Speed_set;
                Motor_2Target = (int16)Speed_set;
                Encoder_Begin(ENCODER_MOTOR_1);
                Circle_status = CIRCLE_RIGHT_END;
                Gyroscope_End(Circle_measureType);
            }
        }
    }
    else if (Circle_status == CIRCLE_RIGHT_END) {
        if (Encoder_sum_Motor1 > Circle_encoderExitRight_Thre) {
            Elec_pidStatus = 1;
            Encoder_End(ENCODER_MOTOR_1);
            if (Circle_multiCircle_Status == 0) {
                Circle_status = CIRCLE_ONLY_ONE;
                if (Circle_speedAcc_Status == 1) {
                    Speed_set += Speed_accVar;
                }
            }
            else {
                Circle_status = CIRCLE_NONE;
            }
        }
    }
}




/*
 * @brief               在环岛进行寻迹(使用摄像头)
 * @attention           使用到了陀螺仪,记得初始化
 * @example
 */
void Circle_RunCamera() {
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
        Trace_traceType = TRACE_Camera_RIGHT;

        //先丢左线后右线
        if (Image_rptsLeftsNum < 0.2 / Image_sampleDist) {++none_left_line;Trace_Status=TRACE_LEFTLOST;}
        if (Image_rptsLeftsNum > 1.0 / Image_sampleDist && none_left_line > 2) {
            ++have_left_line;
            if (have_left_line > 1) {
                Circle_status = CIRCLE_LEFT_IN;
                none_left_line = 0;
                have_left_line = 0;
                Gyroscope_Begin(Circle_measureType);
            }
        }
    }
    //入环,寻内圆左线
    else if (Circle_status == CIRCLE_LEFT_IN) {
        Trace_traceType = TRACE_Camera_LEFT;
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            //这里暂定为>= 90.0,实际的测量可能是负值,这里需要做一下改变
            if (Gyro_x >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_LEFT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
            if (Gyro_y >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_LEFT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (Gyro_z >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_LEFT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
    }
    //在环内,正常巡线 (先做摄像头巡线)
    else if (Circle_status == CIRCLE_LEFT_RUNNING) {
        Trace_traceType = TRACE_Camera_RIGHT;

        //当找到右L角点的时候
        if (Image_LptRight_Found) Image_rptsRightsNum = Image_rptsRightcNum = Image_LptRight_rptsRights_id;
        //外环拐点
        if (Image_LptRight_Found && Image_LptRight_rptsRights_id < 0.4 / Image_sampleDist)
            Circle_status = CIRCLE_LEFT_OUT;
    }
    //出环,寻内圆
    else if (Circle_status == CIRCLE_LEFT_OUT) {
        Trace_traceType = TRACE_Camera_LEFT;

        //右线为长直道
        if (Image_isStraightRight) {
            Circle_status = CIRCLE_LEFT_END;
        }
    }
    //走出圆环,寻右线(用电磁跑)
    else if (Circle_status == CIRCLE_LEFT_END) {
        Trace_traceType = TRACE_Camera_RIGHT;

        //左线先丢后有
        if (Image_rptsLeftsNum < 0.2 / Image_sampleDist)
            ++none_left_line;
        if (Image_rptsLeftsNum > 1 / Image_sampleDist && none_left_line) {
            Circle_status = CIRCLE_NONE;
            none_left_line = 0;
        }
    }

    //------------------------------处理右环岛------------------------------
    //------------------------------
    //1. 开始,寻左边线
    //2. 入环,寻内圆边线
    //3. 在环内,正常巡线(直接用电磁 - 或者用摄像头寻外圆右线)
    //4. 出环,寻内圆
    //5. 走出圆环,寻左线(或者用电磁跑)
    //------------------------------
    //1. 开始,寻左边线
    else if (Circle_status == CIRCLE_RIGHT_BEGIN) {
        Trace_traceType = TRACE_Camera_LEFT;
        //先丢右线后有线
        if (Image_rptsRightsNum < 0.2 / Image_sampleDist){
            ++none_right_line;
            Trace_Status=TRACE_RIGHTLOST;
        }
        if (Image_rptsRightsNum > 1.0 / Image_sampleDist && none_right_line > 2) {
            ++have_right_line;
            if (have_right_line > 1) {
                Circle_status = CIRCLE_RIGHT_IN;
                none_right_line = 0;
                have_right_line = 0;
                Gyroscope_Begin(Circle_measureType);
            }
        }
    }
    //2. 入环,寻内圆边线
    else if (Circle_status == CIRCLE_RIGHT_IN) {
        Trace_traceType = TRACE_Camera_RIGHT;
        //----------------------------------------
        //陀螺仪计数
        if (Circle_measureType == GYROSCOPE_GYRO_X) {
            //这里暂定为>= 90.0,实际的测量可能是负值,这里需要做一下改变
            if (Gyro_x >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_RIGHT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Y) {
            if (Gyro_y >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_RIGHT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
        else if (Circle_measureType == GYROSCOPE_GYRO_Z) {
            if (Gyro_z >= 90.0 || Image_rptsLeftsNum < 0.1 / Image_sampleDist) {
                Circle_status = CIRCLE_RIGHT_RUNNING;
                Gyroscope_End(Circle_measureType);
            }
        }
    }
    //3. 在环内,正常巡线(直接用电磁 - 后者用摄像头寻外圆右线)
    else if (Circle_status == CIRCLE_RIGHT_RUNNING) {
        Trace_traceType = TRACE_Camera_LEFT;
        //当找到右L角点的时候
        if (Image_LptLeft_Found) Image_rptsLeftsNum = Image_rptsLeftcNum = Image_LptLeft_rptsLefts_id;
        //外环拐点
        if (Image_LptLeft_Found && Image_LptLeft_rptsLefts_id < 0.4 / Image_sampleDist)
            Circle_status = CIRCLE_LEFT_OUT;
    }
    //4. 出环,寻内圆
    else if (Circle_status == CIRCLE_RIGHT_OUT) {
        Trace_traceType = TRACE_Camera_RIGHT;
        if (Image_isStraightLeft)
            Circle_status = CIRCLE_RIGHT_END;
    }
    //5. 走出圆环,寻左线(或者用电磁跑)
    else if (Circle_status == CIRCLE_RIGHT_END) {
        Trace_traceType = TRACE_Camera_LEFT;
        //右线先丢后有
        if (Image_rptsRightsNum < 0.2 / Image_sampleDist)
            ++none_right_line;
        if (Image_rptsRightsNum > 0.2 / Image_sampleDist && none_right_line > 2) {
            Circle_status = CIRCLE_NONE;
            none_right_line = 0;
        }
    }
}


/*
 * @brief                   处理环岛
 * @parameter checkMethod   检测入环的方法
 * @parameter runMethod     环内寻迹的方法
 * @attention               1. 环岛积分进入左环岛 - 使用编码器2
 *                             环岛积分进入右环岛 - 使用编码器1
 * @example
 */
void Circle_Handler(CIRCLE_CHECK_METHOD checkMethod, CIRCLE_RUN_METHOD runMethod, CIRCLE_PRE_ENTRY_METHOD entryMethod) {
    //----------------------------------------
    //检测是否入环
    //仅使用摄像头检测
    if (Circle_status == CIRCLE_NONE) {
        if (checkMethod == CIRCLE_CHECK_CAMERA) {
            Circle_CheckCamera();
            if (Circle_leftBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_LEFT_BEGIN;
                Circle_leftBeginStatus_Camera = 0;
                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
                    Encoder_Begin(ENCODER_MOTOR_2);
                }
            }
            else if (Circle_rightBeginStatus_Camera == 1) {
                Circle_status = CIRCLE_RIGHT_BEGIN;
                Circle_rightBeginStatus_Camera = 0;
                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
                    Encoder_Begin(ENCODER_MOTOR_1);
                }
            }
        }
        //仅使用电磁检测
//        else if (checkMethod == CIRCLE_CHECK_ELEC) {
//            Circle_CheckElec();
//            if (Circle_leftBeginStatus_Elec == 1) {
//                Circle_status = CIRCLE_LEFT_BEGIN;
//                Circle_leftBeginStatus_Elec = 0;
//                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
//                    Encoder_Begin(ENCODER_MOTOR_2);
//                }
//            }
//            else if (Circle_rightBeginStatus_Elec == 1) {
//                Circle_status = CIRCLE_RIGHT_BEGIN;
//                Circle_rightBeginStatus_Elec = 0;
//                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
//                    Encoder_Begin(ENCODER_MOTOR_1);
//                }
//            }
//        }
        //二者同时使用
//        else if (checkMethod == CIRCLE_CHECK_BOTH) {
//            Circle_CheckCamera();
//            Circle_CheckElec();
//            if (Circle_leftBeginStatus_Camera == 1 && Circle_leftBeginStatus_Elec == 1) {
//                Circle_status = CIRCLE_LEFT_BEGIN;
//                Circle_leftBeginStatus_Camera = 0;
//                Circle_leftBeginStatus_Elec = 0;
//                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
//                    Encoder_Begin(ENCODER_MOTOR_2);
//                }
//            }
//            else if (Circle_rightBeginStatus_Camera == 1 && Circle_rightBeginStatus_Elec == 1) {
//                Circle_status = CIRCLE_RIGHT_BEGIN;
//                Circle_rightBeginStatus_Camera = 0;
//                Circle_rightBeginStatus_Elec = 0;
//                if (entryMethod == CIRCLE_ENTRY_ENCODER) {
//                    Encoder_Begin(ENCODER_MOTOR_1);
//                }
//            }
//        }
    }

    //----------------------------------------
    //进入圆环进行处理
    if (runMethod == CIRCLE_RUN_CAMERA) {
        Circle_RunCamera();
    }
//    else if (runMethod == CIRCLE_RUN_ELEC) {
//        Circle_RunElec(entryMethod);
//    }
//    else if (runMethod == CIRCLE_RUN_BOTH) {
//        Circle_RunBoth();
//    }
}
void Circle_RunGyscopAndEncoder(){

}

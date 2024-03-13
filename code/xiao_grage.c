
/*
 * xiao_grage.c
 *
 *  Created on: 2023年7月7日
 *      Author: Jayden_NaN
 */

#include "xiao_grage.h"
//------------------------------状态机------------------------------
GRAGE_STATUS Grage_grageStatus = GRAGE_NONE;
uint8 Grage_isDeparture = 0;
uint8 Grage_outWarehous_Status = 0;
//------------------------------基本变量------------------------------
GYROSCOPE_MEASURE_TYPE Grage_measureType = GYROSCOPE_GYRO_X;        //测x的角度(假定,需要测试)
//------------------------------入库变量------------------------------
//--------------------磁铁检测--------------------
uint8 Grage_detectMagnet_scCnt = 0;                 //入库检测成功次数计数
uint8 Grage_detectMagnet_nscCnt = 0;                //入库检测不成功次数计数
uint8 Grage_detectMagnetScCnt_Thre = 5;             //入库检测成功次数阈值
uint8 Grage_detectMagnetNscCnt_Thre = 5;            //入库检测不成功次数阈值
//--------------------摄像头检测--------------------
uint8 (*Grage_detectCamera_rpts)[2];                //挂载中线数据
uint8 Grage_detectCamera_rpts_num = 0;              //记录中线长度
uint8 Grage_zebraFlagBegin = 0;                     //是否在斑马线上标志位
uint8 Grage_zebraFlag0[30];                         //用来存放斑马线的色块数 - 中线的一侧
uint8 Grage_zebraFlag0Num = 0;                      //斑马线色块数         - 中线的一侧
uint8 Grage_zebraFlag1[30];                         //用来存放斑马线的色块数 - 中线的一侧
uint8 Grage_zebraFlag1Num = 0;                      //斑马线色块数         - 中线的一侧

//--------------------入库操作--------------------
int8 Grage_stroageSpeedVar_Motor1_Left = 13;        //入库时强制打角,电机1的变化量 - 入左库
int8 Grage_stroageSpeedVar_Motor2_Left = -13;         //入库时强制打角,电机2的变化量 - 入左库
int8 Grage_stroageSpeedVar_Motor1_Right = -13;        //入库时强制打角,电机1的变化量 - 入右库
int8 Grage_stroageSpeedVar_Motor2_Right = 13;       //入库时强制打角,电机2的变化量 - 入右库
int8 Grage_storageSpeedVarEnd_Motor1 = -12;
int8 Grage_storageSpeedVarEnd_Motor2 = 12;
float Grage_inAngle_Thre = 30.0;                    //入库角度阈值
int16 Grage_inStraight_Thre = 4500;                 //入库直走阈值
//------------------------------出库变量------------------------------
int8 Grage_outSpeedVar_Motor1 = 10;                 //出库时强制打角,电机1的变化量
int8 Grage_outSpeedVar_Motor2 = -10;                //出库时强制打角,电机2的变化量
float Grage_outAngle_Thre = 45.0;                   //出库角度阈值
int8 Grage_outSpeed_Motor1 = 35;
int8 Grage_outSpeed_Motor2 = 25;


/*
 * @brief               车库初始化
 * @attention           发车使用引脚P11_3
 *                      停车使用引脚P32_4
 */
void Grage_Init(void) {
    gpio_init(GRAGE_OUT_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(GRAGE_PART_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

void Grage_Storage_Check(GRAGE_DETECTION_MODE detectMode) {
    //------------------------------磁铁检测------------------------------
    if (detectMode == GRAGE_MAGNET) {
        //检测到信号
        if (!gpio_get_level(GRAGE_PART_PIN)) {
            Elec_pidStatus = 0;


//            ++Grage_detectMagnet_scCnt;
//            if (Grage_detectMagnet_scCnt > Grage_detectMagnetScCnt_Thre) {
//                Grage_detectMagnet_scCnt = 0;
//                Gyroscope_Begin(Grage_measureType);
//            }
            Grage_grageStatus = GRAGE_IN_BEGIN_LEFT;
            Gyroscope_Begin(Grage_measureType);
            Motor_1Target = 60 + Grage_stroageSpeedVar_Motor1_Left;
            Motor_2Target = 60 + Grage_stroageSpeedVar_Motor2_Left;
        }
    }
    //------------------------------摄像头检测------------------------------
    else if (detectMode == GRAGE_CAMERA) {
        if (Grage_grageStatus == GRAGE_NONE) {
            if ((Image_LineIsClosed(0) == true && Image_LineIsClosed(1) == true) ||
                    (Image_iptsLeftNum != 0 && Image_iptsRightNum != 0 &&
                            abs(Image_iptsLeft[0][0] - Image_iptsRight[0][0]) < 20)) {
               // Beep_Tweet();
                Elec_pidStatus = 0;
                Grage_grageStatus = GRAGE_IN_BEGIN_LEFT;
                Motor_1Target = Speed_set + Grage_stroageSpeedVar_Motor1_Left;
                Motor_2Target = Speed_set + Grage_stroageSpeedVar_Motor2_Left;
                Gyroscope_Begin(Grage_measureType);
            }
        }
/*
        if (Grage_grageStatus == GRAGE_NONE) {
            if (Image_LptLeft_Found && !Image_LptRight_Found){
                //仅左角点 - 即左库的情况
                Grage_detectCamera_rpts = Image_rptsLeftc;
                Grage_detectCamera_rpts_num = Image_rptsLeftcNum;
            }
            else if (Image_LptRight_Found && !Image_LptLeft_Found) {
                //仅有右角点 - 即右库的情况
                Grage_detectCamera_rpts = Image_rptsRightc;
                Grage_detectCamera_rpts_num = Image_rptsRightNum;
            }
            else {
                //其余情况不做处理
                Grage_detectCamera_rpts_num = 0;
            }

            uint8 pt[2] = {0, 0};

            //没有能找的中间就退出
            if (Grage_detectCamera_rpts_num == 0)
                return;
            //在中线上一定范围内找斑马线
            for (uint8 i = 10; i < bf_min(80, Grage_detectCamera_rpts_num); ++i) {
                Camera_GetInverse(Grage_detectCamera_rpts[i][0], Grage_detectCamera_rpts[i][1], pt);
                //反变换后图像超出范围则跳过
                if (pt[0] == 0 && pt[1] == 0) {
                    return;
                }
                Grage_zebraFlagBegin = IMAGE_AT(mt9v03x_image[0], pt[0], pt[1]) > image_thre;

                //向中线一侧找
                memset(Grage_zebraFlag0, 0, sizeof(Grage_zebraFlag0));
                Grage_zebraFlag0Num = 0;
                for (int16 x = pt[0] - 1; x >= bf_max(0, pt[0] - 50); --x) {
                    if (Grage_zebraFlagBegin == 0) {
                        //偶数是白色,奇数是黑色
                        if (Grage_zebraFlag0Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //当前是偶数,当前是白色
                            Grage_zebraFlag0[Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //当前是偶数,当前是黑色
                            Grage_zebraFlag0[++Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //当前是奇数,当前是白色
                            Grage_zebraFlag0[++Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //当前是奇数,当前是黑色
                            Grage_zebraFlag0[Grage_zebraFlag0Num]++;
                        }
                    }
                    else {
                        //偶数是黑色,奇数是白色
                        if (Grage_zebraFlag0Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //当前是偶数,当前是白色
                            Grage_zebraFlag0[++Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //当前是偶数,当前是黑色
                            Grage_zebraFlag0[Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //当前是奇数,当前是白色
                            Grage_zebraFlag0[Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //当前是奇数,当前是黑色
                            Grage_zebraFlag0[++Grage_zebraFlag0Num]++;
                        }
                    }
                }

                //向中线的另一侧找
                memset(Grage_zebraFlag1, 0, sizeof(Grage_zebraFlag0));
                Grage_zebraFlag1Num = 0;
                for (int16 x = pt[0] + 1; x <= bf_min(MT9V03X_W - 1, pt[0] + 50); ++x) {
                    if (Grage_zebraFlagBegin == 0) {
                        //偶数是白色,奇数是黑色
                        if (Grage_zebraFlag1Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //当前是偶数,当前是白色
                            Grage_zebraFlag1[Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //当前是偶数,当前是黑色
                            Grage_zebraFlag1[++Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //当前是奇数,当前是白色
                            Grage_zebraFlag1[++Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //当前是奇数,当前是黑色
                            Grage_zebraFlag1[Grage_zebraFlag1Num]++;
                        }

                    }
                    else {
                        //偶数是黑色,奇数是白色
                        if (Grage_zebraFlag1Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //当前是偶数,当前是白色
                            Grage_zebraFlag1[++Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //当前是偶数,当前是黑色
                            Grage_zebraFlag1[Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //当前是奇数,当前是白色
                            Grage_zebraFlag1[Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //当前是奇数,当前是黑色
                            Grage_zebraFlag1[++Grage_zebraFlag1Num]++;
                        }
                    }
                }

                //判断连续跳变的阈值条件以识别斑马线
                uint8 i0 = 1;
                for (; i0 < Grage_zebraFlag0Num -1; ++i0) {
                    if (Grage_zebraFlag0[i0] < 2 || Grage_zebraFlag0[i0] >= 20 || abs(Grage_zebraFlag0[i0 + 1] - Grage_zebraFlag0[i0]) > 10)
                        break;
                }
                uint8 is_zebra0 = i0 > 6;

                uint8 i1 = 1;
                for (; i1 < Grage_zebraFlag1Num -1; ++i1) {
                    if (Grage_zebraFlag1[i1] < 2 || Grage_zebraFlag1[i1] >= 20 || abs(Grage_zebraFlag1[i1 + 1] - Grage_zebraFlag0[i1]) > 10)
                        break;
                }
                uint8 is_zebra1 = i1 > 6;
                if (is_zebra0 && is_zebra1) {
                    Grage_grageStatus = GRAGE_IN_BEGIN_LEFT;
                    Beep_Tweet();
                }
            }
        }
*/
    }
}

void Grage_Departure_Check(void) {

    /*if ((!gpio_get_level(GRAGE_OUT_PIN)) && Grage_isDeparture == 0) {
        Grage_grageStatus = GRAGE_OUT_BEGINT;
    }*/
    if (Grage_outWarehous_Status == 1 && Grage_isDeparture == 0) {
        Grage_grageStatus = GRAGE_OUT_BEGINT;
        Grage_outWarehous_Status = 0;
    }
}

void Grage_Storage(void) {
    if (Grage_grageStatus == GRAGE_IN_BEGIN_LEFT) {
        if (Grage_measureType == GYROSCOPE_GYRO_X) {
            if (fabs(Gyro_x) > Grage_inAngle_Thre) {
                Gyroscope_End(Grage_measureType);
                Grage_grageStatus = GRAGE_IN_GO_STRAIGHT_LEFT;
            }
        }
        else if (Grage_measureType == GYROSCOPE_GYRO_Y) {
            if (fabs(Gyro_y) > Grage_inAngle_Thre) {
                Gyroscope_End(Grage_measureType);
                Grage_grageStatus = GRAGE_IN_GO_STRAIGHT_LEFT;
            }
        }
        else if (Grage_measureType == GYROSCOPE_GYRO_Z) {
            if (fabs(Gyro_z) > Grage_inAngle_Thre) {
                Gyroscope_End(Grage_measureType);
                Grage_grageStatus = GRAGE_IN_GO_STRAIGHT_LEFT;
            }
        }
    }
    else if (Grage_grageStatus == GRAGE_IN_BEGIN_RIGHT) {
        Motor_1Target = Speed_set + Grage_stroageSpeedVar_Motor1_Right;
        Motor_2Target = Speed_set + Grage_stroageSpeedVar_Motor2_Right;
        if (Grage_measureType == GYROSCOPE_GYRO_X) {
            if (fabs(Gyro_x) > Grage_inAngle_Thre) {
                Gyroscope_End(Grage_measureType);
                Grage_grageStatus = GRAGE_IN_GO_STRAIGHT_RIGHT;
            }
        }
        else if (Grage_measureType == GYROSCOPE_GYRO_Y) {
            if (fabs(Gyro_y) > Grage_inAngle_Thre) {
                Gyroscope_End(Grage_measureType);
                Grage_grageStatus = GRAGE_IN_GO_STRAIGHT_RIGHT;
            }
        }
        else if (Grage_measureType == GYROSCOPE_GYRO_Z) {
            if (fabs(Gyro_z) > Grage_inAngle_Thre) {
                Gyroscope_End(Grage_measureType);
                Grage_grageStatus = GRAGE_IN_GO_STRAIGHT_RIGHT;
            }
        }
    }
    else if (Grage_grageStatus  == GRAGE_IN_GO_STRAIGHT_LEFT) {
        Encoder_Begin(ENCODER_MOTOR_1);
        Elec_pidStatus = 0;
        Motor_1Target = Speed_set / 2;
        Motor_2Target = Speed_set / 2;
        Grage_grageStatus = GRAGE_IN_END_LEFT;
    }
    else if (Grage_grageStatus == GRAGE_IN_GO_STRAIGHT_RIGHT) {
        Encoder_Begin(ENCODER_MOTOR_2);
        Motor_1Target = Speed_set / 2;
        Motor_2Target = Speed_set / 2;
        Grage_grageStatus = GRAGE_IN_END_RIGHT;
    }
    else if (Grage_grageStatus == GRAGE_IN_END_LEFT) {
        if (abs(Encoder_sum_Motor1) > Grage_inStraight_Thre) {
            Motor_1Target = 0;
            Motor_2Target = 0;
            Grage_isDeparture = 0;
            Encoder_End(ENCODER_MOTOR_1);
            Grage_outWarehous_Status = 0;
            Grage_grageStatus = GRAGE_NONE;
        }
    }
    else if (Grage_grageStatus == GRAGE_IN_END_RIGHT) {
        if (abs(Encoder_sum_Motor2) > Grage_inStraight_Thre) {
            Elec_pidStatus = 0;
            Motor_1Target = 0;
            Motor_2Target = 0;
            Grage_isDeparture = 0;
            Encoder_End(ENCODER_MOTOR_2);
            Grage_outWarehous_Status = 0;
            Grage_grageStatus = GRAGE_NONE;
        }

    }
    else if (Grage_grageStatus == GRAGE_OUT_BEGINT) {
        Elec_pidStatus = 0;
        Motor_pidStatus = 1;
        Motor_1Target = Grage_outSpeedVar_Motor1;
        Motor_2Target = Grage_outSpeedVar_Motor2;
        Gyroscope_Begin(Grage_measureType);
        Grage_grageStatus = GRAGE_OUT_END;
    }
    else if (Grage_grageStatus == GRAGE_OUT_END) {
        if (Grage_measureType == GYROSCOPE_GYRO_X) {
            if (fabs(Gyro_x) > Grage_outAngle_Thre) {
                Gyroscope_End(Grage_measureType);
                Grage_grageStatus = GRAGE_NONE;
                Elec_pidStatus = 1;
                Grage_isDeparture = 1;
            //    Beep_Tweet();
            }
        }
        else if (Grage_measureType == GYROSCOPE_GYRO_Y) {
            if (fabs(Gyro_y) > Grage_outAngle_Thre) {
                Gyroscope_End(Grage_measureType);
                Grage_grageStatus = GRAGE_NONE;
                Elec_pidStatus = 1;
            }
        }
        else if (Grage_measureType == GYROSCOPE_GYRO_Z) {
            if (fabs(Gyro_z) > Grage_outAngle_Thre) {
                Gyroscope_End(Grage_measureType);
                Grage_grageStatus = GRAGE_NONE;
                Elec_pidStatus = 1;
            }
        }
    }
}


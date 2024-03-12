
/*
 * xiao_grage.c
 *
 *  Created on: 2023��7��7��
 *      Author: Jayden_NaN
 */

#include "xiao_grage.h"
//------------------------------״̬��------------------------------
GRAGE_STATUS Grage_grageStatus = GRAGE_NONE;
uint8 Grage_isDeparture = 0;
uint8 Grage_outWarehous_Status = 0;
//------------------------------��������------------------------------
GYROSCOPE_MEASURE_TYPE Grage_measureType = GYROSCOPE_GYRO_X;        //��x�ĽǶ�(�ٶ�,��Ҫ����)
//------------------------------������------------------------------
//--------------------�������--------------------
uint8 Grage_detectMagnet_scCnt = 0;                 //�����ɹ���������
uint8 Grage_detectMagnet_nscCnt = 0;                //����ⲻ�ɹ���������
uint8 Grage_detectMagnetScCnt_Thre = 5;             //�����ɹ�������ֵ
uint8 Grage_detectMagnetNscCnt_Thre = 5;            //����ⲻ�ɹ�������ֵ
//--------------------����ͷ���--------------------
uint8 (*Grage_detectCamera_rpts)[2];                //������������
uint8 Grage_detectCamera_rpts_num = 0;              //��¼���߳���
uint8 Grage_zebraFlagBegin = 0;                     //�Ƿ��ڰ������ϱ�־λ
uint8 Grage_zebraFlag0[30];                         //������Ű����ߵ�ɫ���� - ���ߵ�һ��
uint8 Grage_zebraFlag0Num = 0;                      //������ɫ����         - ���ߵ�һ��
uint8 Grage_zebraFlag1[30];                         //������Ű����ߵ�ɫ���� - ���ߵ�һ��
uint8 Grage_zebraFlag1Num = 0;                      //������ɫ����         - ���ߵ�һ��

//--------------------������--------------------
int8 Grage_stroageSpeedVar_Motor1_Left = 13;        //���ʱǿ�ƴ��,���1�ı仯�� - �����
int8 Grage_stroageSpeedVar_Motor2_Left = -13;         //���ʱǿ�ƴ��,���2�ı仯�� - �����
int8 Grage_stroageSpeedVar_Motor1_Right = -13;        //���ʱǿ�ƴ��,���1�ı仯�� - ���ҿ�
int8 Grage_stroageSpeedVar_Motor2_Right = 13;       //���ʱǿ�ƴ��,���2�ı仯�� - ���ҿ�
int8 Grage_storageSpeedVarEnd_Motor1 = -12;
int8 Grage_storageSpeedVarEnd_Motor2 = 12;
float Grage_inAngle_Thre = 30.0;                    //���Ƕ���ֵ
int16 Grage_inStraight_Thre = 4500;                 //���ֱ����ֵ
//------------------------------�������------------------------------
int8 Grage_outSpeedVar_Motor1 = 10;                 //����ʱǿ�ƴ��,���1�ı仯��
int8 Grage_outSpeedVar_Motor2 = -10;                //����ʱǿ�ƴ��,���2�ı仯��
float Grage_outAngle_Thre = 45.0;                   //����Ƕ���ֵ
int8 Grage_outSpeed_Motor1 = 35;
int8 Grage_outSpeed_Motor2 = 25;


/*
 * @brief               �����ʼ��
 * @attention           ����ʹ������P11_3
 *                      ͣ��ʹ������P32_4
 */
void Grage_Init(void) {
    gpio_init(GRAGE_OUT_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(GRAGE_PART_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

void Grage_Storage_Check(GRAGE_DETECTION_MODE detectMode) {
    //------------------------------�������------------------------------
    if (detectMode == GRAGE_MAGNET) {
        //��⵽�ź�
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
    //------------------------------����ͷ���------------------------------
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
                //����ǵ� - ���������
                Grage_detectCamera_rpts = Image_rptsLeftc;
                Grage_detectCamera_rpts_num = Image_rptsLeftcNum;
            }
            else if (Image_LptRight_Found && !Image_LptLeft_Found) {
                //�����ҽǵ� - ���ҿ�����
                Grage_detectCamera_rpts = Image_rptsRightc;
                Grage_detectCamera_rpts_num = Image_rptsRightNum;
            }
            else {
                //���������������
                Grage_detectCamera_rpts_num = 0;
            }

            uint8 pt[2] = {0, 0};

            //û�����ҵ��м���˳�
            if (Grage_detectCamera_rpts_num == 0)
                return;
            //��������һ����Χ���Ұ�����
            for (uint8 i = 10; i < bf_min(80, Grage_detectCamera_rpts_num); ++i) {
                Camera_GetInverse(Grage_detectCamera_rpts[i][0], Grage_detectCamera_rpts[i][1], pt);
                //���任��ͼ�񳬳���Χ������
                if (pt[0] == 0 && pt[1] == 0) {
                    return;
                }
                Grage_zebraFlagBegin = IMAGE_AT(mt9v03x_image[0], pt[0], pt[1]) > image_thre;

                //������һ����
                memset(Grage_zebraFlag0, 0, sizeof(Grage_zebraFlag0));
                Grage_zebraFlag0Num = 0;
                for (int16 x = pt[0] - 1; x >= bf_max(0, pt[0] - 50); --x) {
                    if (Grage_zebraFlagBegin == 0) {
                        //ż���ǰ�ɫ,�����Ǻ�ɫ
                        if (Grage_zebraFlag0Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //��ǰ��ż��,��ǰ�ǰ�ɫ
                            Grage_zebraFlag0[Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //��ǰ��ż��,��ǰ�Ǻ�ɫ
                            Grage_zebraFlag0[++Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //��ǰ������,��ǰ�ǰ�ɫ
                            Grage_zebraFlag0[++Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //��ǰ������,��ǰ�Ǻ�ɫ
                            Grage_zebraFlag0[Grage_zebraFlag0Num]++;
                        }
                    }
                    else {
                        //ż���Ǻ�ɫ,�����ǰ�ɫ
                        if (Grage_zebraFlag0Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //��ǰ��ż��,��ǰ�ǰ�ɫ
                            Grage_zebraFlag0[++Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //��ǰ��ż��,��ǰ�Ǻ�ɫ
                            Grage_zebraFlag0[Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //��ǰ������,��ǰ�ǰ�ɫ
                            Grage_zebraFlag0[Grage_zebraFlag0Num]++;
                        }
                        else if (Grage_zebraFlag0Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //��ǰ������,��ǰ�Ǻ�ɫ
                            Grage_zebraFlag0[++Grage_zebraFlag0Num]++;
                        }
                    }
                }

                //�����ߵ���һ����
                memset(Grage_zebraFlag1, 0, sizeof(Grage_zebraFlag0));
                Grage_zebraFlag1Num = 0;
                for (int16 x = pt[0] + 1; x <= bf_min(MT9V03X_W - 1, pt[0] + 50); ++x) {
                    if (Grage_zebraFlagBegin == 0) {
                        //ż���ǰ�ɫ,�����Ǻ�ɫ
                        if (Grage_zebraFlag1Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //��ǰ��ż��,��ǰ�ǰ�ɫ
                            Grage_zebraFlag1[Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //��ǰ��ż��,��ǰ�Ǻ�ɫ
                            Grage_zebraFlag1[++Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //��ǰ������,��ǰ�ǰ�ɫ
                            Grage_zebraFlag1[++Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //��ǰ������,��ǰ�Ǻ�ɫ
                            Grage_zebraFlag1[Grage_zebraFlag1Num]++;
                        }

                    }
                    else {
                        //ż���Ǻ�ɫ,�����ǰ�ɫ
                        if (Grage_zebraFlag1Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //��ǰ��ż��,��ǰ�ǰ�ɫ
                            Grage_zebraFlag1[++Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 0 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //��ǰ��ż��,��ǰ�Ǻ�ɫ
                            Grage_zebraFlag1[Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) > image_thre) {
                            //��ǰ������,��ǰ�ǰ�ɫ
                            Grage_zebraFlag1[Grage_zebraFlag1Num]++;
                        }
                        else if (Grage_zebraFlag1Num % 2 == 1 && IMAGE_AT(mt9v03x_image[0], x, pt[1]) < image_thre) {
                            //��ǰ������,��ǰ�Ǻ�ɫ
                            Grage_zebraFlag1[++Grage_zebraFlag1Num]++;
                        }
                    }
                }

                //�ж������������ֵ������ʶ�������
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


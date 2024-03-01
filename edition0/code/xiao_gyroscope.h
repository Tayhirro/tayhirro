/*
 * xiao_gyroscope.h
 *
 *  Created on: 2023��7��2��
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_GYROSCOPE_H_
#define CODE_XIAO_GYROSCOPE_H_

#include "xiao_common.h"

//------------------------------�������Ͷ���------------------------------
//ʹ�õ�����������
typedef enum {
    GYROSCOPE_IMU660RA = 0x00,
    GYROSCOPE_IMU963RA = 0x01,
    GYROSCOPE_ICM20602 = 0x02,
}GYROSCOPE_TYPE;

//�����ǲ���������
typedef enum {
    GYROSCOPE_GYRO_X = 0x00,
    GYROSCOPE_GYRO_Y = 0x01,
    GYROSCOPE_GYRO_Z = 0x02,
    GYROSCOPE_ACC_X = 0x03,
    GYROSCOPE_ACC_Y = 0x04,
    GYROSCOPE_ACC_Z = 0x05,
}GYROSCOPE_MEASURE_TYPE;

//������ƫ��������
struct GyroscopeOffset {
        float Gyro_Xdata;
        float Gyro_Ydata;
        float Gyro_Zdata;
        float ACC_Xdata;
        float ACC_Ydata;
        float ACC_Zdata;
};


//------------------------------�������������õ�����------------------------------
extern float Gyro_x;           //������xֵ
extern float Gyro_y;           //������yֵ
extern float Gyro_z;           //������zֵ
extern float Acc_x;            //���ٶ�xֵ
extern float Acc_y;            //���ٶ�yֵ
extern float Acc_z;            //���ٶ�zֵ

extern float Gyro_corrX;       //������xֵ - ���ٶ�
extern float Gyro_corrY;       //������yֵ - ���ٶ�
extern float Gyro_corrZ;       //������zֵ - ���ٶ�
extern float Acc_corrX;        //���ٶ�xֵ - ���ٶ�
extern float Acc_corrY;        //���ٶ�yֵ - ���ٶ�
extern float Acc_corrZ;        //���ٶ�zֵ - ���ٶ�

//------------------------------����------------------------------
void Gyroscope_Init(GYROSCOPE_TYPE device, uint16 time);
void Gyroscope_Begin(GYROSCOPE_MEASURE_TYPE measureType);
void Gyroscope_End(GYROSCOPE_MEASURE_TYPE measureType);
void Gyroscope_GetData(void);
void Gyroscope_Conut(void);
void Gyroscope_Clear(GYROSCOPE_MEASURE_TYPE measureType);


#endif /* CODE_XIAO_GYROSCOPE_H_ */

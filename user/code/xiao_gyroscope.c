/*
 * xiao_gyroscope.c
 *
 *  Created on: 2023��7��2��
 *      Author: Jayden_NaN
 */


#include "xiao_gyroscope.h"

//------------------------------��������------------------------------
//��ʱʱ�� - ��λ ms
uint16 Gyroscope_time = 0;
//ʹ�õ��������豸(Ĭ������ΪIMU660RA)
GYROSCOPE_TYPE Gyroscope_device = GYROSCOPE_IMU660RA;
//������ƫ����
struct GyroscopeOffset Gyro_Offset;

//------------------------------��������------------------------------
//------------------------------
//�洢���ݱ���
float Gyro_x;           //������xֵ - �Ƕ�
float Gyro_y;           //������yֵ - �Ƕ�
float Gyro_z;           //������zֵ - �Ƕ�
float Acc_x;            //���ٶ�xֵ - �ٶ�
float Acc_y;            //���ٶ�yֵ - �ٶ�
float Acc_z;            //���ٶ�zֵ - �ٶ�

float Gyro_corrX;       //������xֵ - ���ٶ�
float Gyro_corrY;       //������yֵ - ���ٶ�
float Gyro_corrZ;       //������zֵ - ���ٶ�
float Acc_corrX;        //���ٶ�xֵ - ���ٶ�
float Acc_corrY;        //���ٶ�yֵ - ���ٶ�
float Acc_corrZ;        //���ٶ�zֵ - ���ٶ�

//------------------------------
//����״̬��
uint8 Gyro_x_status = 0;    //������xֵ����״̬��
uint8 Gyro_y_status = 0;    //������yֵ����״̬��
uint8 Gyro_z_status = 0;    //������zֵ����״̬��
uint8 Acc_x_status = 0;     //���ٶ�xֵ����״̬��
uint8 Acc_y_status = 0;     //���ٶ�yֵ����״̬��
uint8 Acc_z_status = 0;     //���ٶ�zֵ����״̬��

/*
 * @brief               �����Ǵ����ʼ��
 * @parameter device    ʹ�õ��������豸
 * @parameter time      ʹ�õ��ж�ʱ��
 * @example             Gyroscope_Init(GYROSCOPE_IMU660RA, 10);
 */
void Gyroscope_Init(GYROSCOPE_TYPE device, uint16 time) {
    //������ƫ������ʼ������Ϊ0
    Gyro_Offset.ACC_Xdata = 0.0;
    Gyro_Offset.ACC_Ydata = 0.0;
    Gyro_Offset.ACC_Zdata = 0.0;
    Gyro_Offset.Gyro_Xdata = 0.0;
    Gyro_Offset.Gyro_Ydata = 0.0;
    Gyro_Offset.Gyro_Zdata = 0.0;

    //����IMU660RA�Ĵ���
    if (device == GYROSCOPE_IMU660RA) {
        Gyroscope_device = GYROSCOPE_IMU660RA;
        //���ݵ�ѭ������150��,ÿ����һ��,ϵͳ��ʱ10ms,���Թ���Ҫ��ʱ1.5s
        for (uint8 i = 0; i < 150; ++i) {
            imu660ra_get_gyro();
            imu660ra_get_acc();
            Gyro_Offset.ACC_Xdata += (float)imu660ra_acc_x;
            Gyro_Offset.ACC_Ydata += (float)imu660ra_acc_y;
            Gyro_Offset.ACC_Zdata += (float)imu660ra_acc_z;
            Gyro_Offset.Gyro_Xdata += (float)imu660ra_gyro_x;
            Gyro_Offset.Gyro_Ydata += (float)imu660ra_gyro_y;
            Gyro_Offset.Gyro_Zdata += (float)imu660ra_gyro_z;
            system_delay_ms(10);
        }
            Gyro_Offset.ACC_Xdata /= 150;
            Gyro_Offset.ACC_Ydata /= 150;
            Gyro_Offset.ACC_Zdata /= 150;
            Gyro_Offset.Gyro_Xdata /= 150;
            Gyro_Offset.Gyro_Ydata /= 150;
            Gyro_Offset.Gyro_Zdata /= 150;
    }
    //����IMU963RA�Ĵ���
    else if (device == GYROSCOPE_IMU963RA) {
        Gyroscope_device = GYROSCOPE_IMU963RA;

    }
    //����ICM20602�Ĵ���
    else if (device == GYROSCOPE_ICM20602) {
        Gyroscope_device = GYROSCOPE_ICM20602;
    }
    //���������Ȳ�������,�Լ������õ����ݹ���~~~
    Gyroscope_time = time;
}

/*
 * @brief                   ��ʱ����ʼ����
 * @parameter measureType   ��������
 * @example                 Gyroscope_Begin(GYROSCOPE_GYRO_X);
 *
 */
void Gyroscope_Begin(GYROSCOPE_MEASURE_TYPE measureType) {
    //Ҫ��,ֻ�м�����ת̬Ϊ0��ʱ��,���ܿ�ʼ���м���
    if (measureType == GYROSCOPE_GYRO_X) {
        if (Gyro_x_status == 0) {
            Gyro_x_status = 1;
            Gyro_x = 0.0;
        }
    }
    else if (measureType == GYROSCOPE_GYRO_Y) {
        if (Gyro_y_status == 0) {
            Gyro_y_status = 1;
            Gyro_y = 0.0;
        }
    }
    else if (measureType == GYROSCOPE_GYRO_Z) {
        if (Gyro_z_status == 0) {
            Gyro_z_status = 1;
            Gyro_z = 0.0;
        }
    }
    else if (measureType == GYROSCOPE_ACC_X) {
        if (Acc_x_status == 0) {
            Acc_x_status = 1;
            Acc_x = 0.0;
        }
    }
    else if (measureType == GYROSCOPE_ACC_Y) {
        if (Acc_y_status == 0) {
            Acc_y_status = 1;
            Acc_y = 0.0;
        }
    }
    else if (measureType == GYROSCOPE_ACC_Z) {
        if (Acc_z_status == 0) {
            Acc_z_status = 1;
            Acc_z = 0.0;
        }
    }
}

/*
 * @brief                   ������ֹͣ����
 * @parameter measureType   ��������
 * @example                 Gyroscope_End(GYROSCOPE_GYRO_X);
 *
 */
void Gyroscope_End(GYROSCOPE_MEASURE_TYPE measureType) {
    //�ص��Ļ���ֱ�Ӱ�״̬��0��,���ÿ����Ƿ����ڹ���״̬
    if (measureType == GYROSCOPE_GYRO_X) {
        Gyro_x_status = 0;
    }
    else if (measureType == GYROSCOPE_GYRO_Y) {
        Gyro_y_status = 0;
    }
    else if (measureType == GYROSCOPE_GYRO_Z) {
        Gyro_z_status = 0;
    }
    else if (measureType == GYROSCOPE_ACC_X) {
        Acc_x_status = 0;
    }
    else if (measureType == GYROSCOPE_ACC_Y) {
        Acc_y_status = 0;
    }
    else if (measureType == GYROSCOPE_ACC_Z) {
        Acc_z_status = 0;
    }
}

void Gyroscope_GetData(void) {
    //------------------------------��������------------------------------
    if (Gyroscope_device == GYROSCOPE_IMU660RA) {
        imu660ra_get_gyro();
        imu660ra_get_acc();
    }
    else if (Gyroscope_device == GYROSCOPE_IMU963RA) {
        imu963ra_get_gyro();
        imu963ra_get_acc();
    }
    else if (Gyroscope_device == GYROSCOPE_ICM20602) {
        icm20602_get_gyro();
        icm20602_get_acc();
    }

    if (Gyroscope_device == GYROSCOPE_IMU660RA) {
        Gyro_corrX = imu660ra_gyro_transition((float)imu660ra_gyro_x - Gyro_Offset.Gyro_Xdata);
        Gyro_corrY = imu660ra_gyro_transition((float)imu660ra_gyro_y - Gyro_Offset.Gyro_Ydata);
        Gyro_corrZ = imu660ra_gyro_transition((float)imu660ra_gyro_z - Gyro_Offset.Gyro_Zdata);
        Acc_corrX = imu660ra_acc_transition((float)imu660ra_acc_x - Gyro_Offset.ACC_Xdata);
        Acc_corrY = imu660ra_acc_transition((float)imu660ra_acc_y - Gyro_Offset.ACC_Ydata);
        Acc_corrZ = imu660ra_acc_transition((float)imu660ra_acc_z - Gyro_Offset.ACC_Zdata);
    }
}



void Gyroscope_Conut(void) {
    //------------------------------����˵��------------------------------
    //���м���,���ж�����������ݵ��ۼ�
    //�ٶȵ�λΪm/s
    //�Ƕȵ�λΪ��
    //--------------------��ȡ����--------------------
    Gyroscope_GetData();

    //--------------------���ݴ���--------------------
    if (Gyro_x_status == 1) {
        if (Gyroscope_device == GYROSCOPE_IMU660RA) {
            //Gyro_x += imu660ra_gyro_transitionFloat((float)imu660ra_gyro_x - Gyro_Offset.Gyro_Xdata) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_IMU963RA) {
            Gyro_x += imu963ra_gyro_transition(imu963ra_gyro_x) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_ICM20602) {
            Gyro_x += icm20602_gyro_transition(icm20602_gyro_x) * Gyroscope_time * 0.001;
        }
    }

    if (Gyro_y_status == 1) {
        if (Gyroscope_device == GYROSCOPE_IMU660RA) {
            //Gyro_y += imu660ra_gyro_transitionFloat((float)imu660ra_gyro_y - Gyro_Offset.Gyro_Ydata) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_IMU963RA) {
            Gyro_y += imu963ra_gyro_transition(imu963ra_gyro_y) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_ICM20602) {
            Gyro_y += icm20602_gyro_transition(icm20602_gyro_y) * Gyroscope_time * 0.001;
        }
    }

    if (Gyro_z_status == 1) {
        if (Gyroscope_device == GYROSCOPE_IMU660RA) {
            //Gyro_z += imu660ra_gyro_transitionFloat((float)imu660ra_gyro_z - Gyro_Offset.Gyro_Zdata) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_IMU963RA) {
            Gyro_z += imu963ra_gyro_transition(imu963ra_gyro_z) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_ICM20602) {
            Gyro_z += icm20602_gyro_transition(icm20602_gyro_z) * Gyroscope_time * 0.001;
        }
    }

    if (Acc_x_status == 1) {
        if (Gyroscope_device == GYROSCOPE_IMU660RA) {
            //Acc_x += imu660ra_acc_transitionFloat((float)imu660ra_acc_x - Gyro_Offset.ACC_Xdata) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_IMU963RA) {
            Acc_x += imu963ra_acc_transition(imu963ra_acc_x) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_ICM20602) {
            Acc_x += icm20602_gyro_transition(icm20602_gyro_x) * Gyroscope_time * 0.001;
        }
    }

    if (Acc_y_status == 1) {
        if (Gyroscope_device == GYROSCOPE_IMU660RA) {
            //Acc_y += imu660ra_acc_transitionFloat((float)imu660ra_acc_y - Gyro_Offset.ACC_Ydata) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_IMU963RA) {
            Acc_y += imu963ra_acc_transition(imu963ra_acc_y) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_ICM20602) {
            Acc_y += icm20602_gyro_transition(icm20602_gyro_y) * Gyroscope_time * 0.001;
        }
    }

    if (Acc_z_status == 1) {
        if (Gyroscope_device == GYROSCOPE_IMU660RA) {
            //Acc_z += imu660ra_acc_transitionFloat((float)imu660ra_acc_z - Gyro_Offset.ACC_Zdata) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_IMU963RA) {
            Acc_z += imu963ra_acc_transition(imu963ra_acc_z) * Gyroscope_time * 0.001;
        }
        else if (Gyroscope_device == GYROSCOPE_ICM20602) {
            Acc_z += icm20602_gyro_transition(icm20602_gyro_z) * Gyroscope_time * 0.001;
        }
    }
}

/*
 * @brief                   ��ջ��ֵ�����
 * @paramter measureType    ����������
 */
void Gyroscope_Clear(GYROSCOPE_MEASURE_TYPE measureType) {
    if (measureType == GYROSCOPE_GYRO_X) {
        Gyro_x = 0.0;
    }
    else if (measureType == GYROSCOPE_GYRO_Y) {
        Gyro_y = 0.0;
    }
    else if (measureType == GYROSCOPE_GYRO_Z) {
        Gyro_z = 0.0;
    }
    else if (measureType == GYROSCOPE_ACC_X) {
        Acc_x = 0.0;
    }
    else if (measureType == GYROSCOPE_ACC_Y) {
        Acc_y = 0.0;
    }
    else if (measureType == GYROSCOPE_ACC_Z) {
        Acc_z = 0.0;
    }
}

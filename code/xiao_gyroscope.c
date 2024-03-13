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

float Gyr_filtX;
float Gyr_filtY;
float Gyr_filtZ;
float Acc_filtX;
float Acc_filtY;
float Acc_filtZ;
float alpha=0.98;
//double halfT=0.00125f;
//------------------------------
//����״̬��
uint8 Gyro_x_status = 0;    //������xֵ����״̬��
uint8 Gyro_y_status = 0;    //������yֵ����״̬��
uint8 Gyro_z_status = 0;    //������zֵ����״̬��
uint8 Acc_x_status = 0;     //���ٶ�xֵ����״̬��
uint8 Acc_y_status = 0;     //���ٶ�yֵ����״̬��
uint8 Acc_z_status = 0;     //���ٶ�zֵ����״̬��

//������Ԫ��
float q0=1.0f;
float q1=0.0f;
float q2=0.0f;
float q3=0.0f;
//������̬��
float pitch=0.0;
float yaw=0.0;
float roll=0.0;
//
//float Kp=1.6f;
//float Ki=0.001f;

#define Kp_New      0.9f                            //�����˲���ǰ���ݵ�Ȩ��
#define Kp_Old      0.1f                            //�����˲���ʷ���ݵ�Ȩ��
#define Acc_Gain    0.0001220f                        //���ٶȱ��G (��ʼ�����ٶ�������-+4g LSBa = 2*4/65535.0)
#define Gyro_Gain   0.0609756f                            //���ٶȱ�ɶ� (��ʼ��������������+-2000 LSBg = 2*2000/65535.0)
#define Gyro_Gr     0.0010641f                            //���ٶȱ�ɻ���(3.1415/180 * LSBg)


float   accb[3];                              //���ٶȼ����ݴ洢
float   gyrb[3];                               //���ٶȼ����ݴ洢
float   DCMgb[3][3];                          //���������󣨽� ��������ϵ ת��Ϊ ��������ϵ��
uint8_t AccbUpdate = 0;                       //���ٶȼ����ݸ��±�־λ
uint8_t GyrbUpdate = 0;                       //���ٶȼ����ݸ��±�־λ
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
    //IMUupdate(imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z, imu660ra_acc_x,imu660ra_acc_y, imu660ra_acc_z, alpha, halfT);

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

void Prepare_Data(void)
{
    //MPU6050_Read();    //������ȡ ����������
    //MPU6050_Offset();  //��MPU6050���д�����ȥ��ƫ�����û�м�����ƫ(���ݱ�־λ�ж�)�ͼ�����ƫ
    Gyroscope_GetData();
    //MPU6050_AccRead(&Acc_filt);//��ȡ���ٶȼ�ԭʼ���ݣ������п������˲�
    //MPU6050_GyroRead(&Gyr_filt);//��ȡ���ٶȼ�ԭʼ���ݣ�������һ�׵�ͨ�˲�

//  Aver_FilterXYZ(&Acc_filt,12);//�˴��Լ��ٶȼƽ��л��������˲�����
//  Aver_FilterXYZ(&Gyr_filt,12);//�˴��Խ��ٶȼƽ��л��������˲�����

    //���ٶ�ADֵ ת���� ��/ƽ����
    Acc_filtX = (float)Acc_corrX * Acc_Gain * G;
    Acc_filtY = (float)Acc_corrY * Acc_Gain * G;
    Acc_filtZ = (float)Acc_corrZ * Acc_Gain * G;
//  printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",Acc_filt.X,Acc_filt.Y,Acc_filt.Z);

    //������ADֵ ת���� ����/��
    Gyr_filtX = (float) Gyro_corrX * Gyro_Gr;
    Gyr_filtY = (float) Gyro_corrX * Gyro_Gr;
    Gyr_filtZ = (float) Gyro_corrX * Gyro_Gr;
//  printf("gx=%0.2f gy=%0.2f gz=%0.2f\r\n",Gyr_filt.X,Gyr_filt.Y,Gyr_filt.Z);
}

/*********************************************************************************************************
* ��  ����void IMUupdate(FLOAT_XYZ *Gyr_filt,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Att_Angle)
* �����ܣ���ȡ��̬��
* ��  ����Gyr_filt     ָ����ٶȵ�ָ�루ע�ⵥλ�����ǻ��ȣ�
*         Acc_filt  ָ����ٶȵ�ָ��
*         Att_Angle ָ����̬�ǵ�ָ��
* ����ֵ����
* ��  ע�������Ԫ����ŷ���Ƕ��ڴ˺��������
**********************************************************************************************************/
//kp=ki=0 ������ȫ����������
#define Kp 1.5f                          // proportional gain governs rate of convergence to accelerometer/magnetometer
                                          //����������Ƽ��ٶȼƣ������Ƶ���������
#define Ki 0.005f                         // integral gain governs rate of convergence of gyroscope biases
                                          //���������������ƫ��������ٶ�
#define halfT 0.005f                      // half the sample period �������ڵ�һ��

//float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

void IMUupdate(double gx, double gy, double gz, double ax, double ay, double az)
{
    //float ax = Acc_filt->X,ay = Acc_filt->Y,az = Acc_filt->Z;
    //float gx = Gyr_filt->X,gy = Gyr_filt->Y,gz = Gyr_filt->Z;
    float vx, vy, vz;
    float ex, ey, ez;
    float norm;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    if(ax*ay*az==0)return;

    //���ٶȼ�<����>���������ٶ�����(��������ϵ)
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
//  printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);

    //�����ǻ���<����>��������(��������ϵ)
    vx = 2*(q1q3 - q0q2);                  //����(3,1)��
    vy = 2*(q0q1 + q2q3);                //����(3,2)��
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;     //����(3,3)��

    // printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz);

    //����������õ�ֵ
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    //��������������л���
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    //�����PI�󲹳���������
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;//�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�

    //��Ԫ�ص�΢�ַ���
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    //��λ����Ԫ��
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

//  ������ʽ
//  matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;   // 11
//  matrix[1] = 2.f * (q1q2 + q0q3);           // 12
//  matrix[2] = 2.f * (q1q3 - q0q2);           // 13
//  matrix[3] = 2.f * (q1q2 - q0q3);           // 21
//  matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;   // 22
//  matrix[5] = 2.f * (q2q3 + q0q1);           // 23
//  matrix[6] = 2.f * (q1q3 + q0q2);           // 31
//  matrix[7] = 2.f * (q2q3 - q0q1);           // 32
//  matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;   // 33

    //��Ԫ��ת����ŷ����(Z->Y->X)

    //ƫ����YAW
    if( (Gyr_filtZ *RadtoDeg > 1.0f) || (Gyr_filtZ *RadtoDeg < -1.0f) ) //����̫С������Ϊ�Ǹ��ţ�����ƫ������
    {
        yaw += Gyr_filtZ *RadtoDeg*0.01f;
    }
//  printf("yaw: %f\r\n",Att_Angle->yaw);//YAW�����ݣ��ǳ��ȶ���

    //�����ROLL
    roll = -asin(2.f * (q1q3 - q0q2))* 57.3f;
//  printf("%f\r\n",Att_Angle->rol);//ROLL������

    //������PITCH
    pitch = -atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f ;
//  printf("pitch: %f\r\n",Att_Angle->pit);//PITCH������
//    printf("%f,%f,%f\r\n",Att_Angle->pit,Att_Angle->rol,Att_Angle->yaw);//PITCH������

}

void imu_work(void){
    Prepare_Data();
    IMUupdate(Gyr_filtX, Gyr_filtY,Gyr_filtZ, Acc_filtX, Acc_filtY,Acc_filtZ);

}

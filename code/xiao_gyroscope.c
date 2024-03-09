/*
 * xiao_gyroscope.c
 *
 *  Created on: 2023年7月2日
 *      Author: Jayden_NaN
 */


#include "xiao_gyroscope.h"

//------------------------------基本变量------------------------------
//计时时间 - 单位 ms
uint16 Gyroscope_time = 0;
//使用的陀螺仪设备(默认设置为IMU660RA)
GYROSCOPE_TYPE Gyroscope_device = GYROSCOPE_IMU660RA;
//陀螺仪偏移量
struct GyroscopeOffset Gyro_Offset;

//------------------------------计数变量------------------------------
//------------------------------
//存储数据变量
float Gyro_x;           //陀螺仪x值 - 角度
float Gyro_y;           //陀螺仪y值 - 角度
float Gyro_z;           //陀螺仪z值 - 角度
float Acc_x;            //加速度x值 - 速度
float Acc_y;            //加速度y值 - 速度
float Acc_z;            //加速度z值 - 速度

float Gyro_corrX;       //陀螺仪x值 - 角速度
float Gyro_corrY;       //陀螺仪y值 - 角速度
float Gyro_corrZ;       //陀螺仪z值 - 角速度
float Acc_corrX;        //加速度x值 - 加速度
float Acc_corrY;        //加速度y值 - 加速度
float Acc_corrZ;        //加速度z值 - 加速度

float Gyr_filtX;
float Gyr_filtY;
float Gyr_filtZ;
float Acc_filtX;
float Acc_filtY;
float Acc_filtZ;
float alpha=0.98;
//double halfT=0.00125f;
//------------------------------
//计数状态机
uint8 Gyro_x_status = 0;    //陀螺仪x值计数状态机
uint8 Gyro_y_status = 0;    //陀螺仪y值计数状态机
uint8 Gyro_z_status = 0;    //陀螺仪z值计数状态机
uint8 Acc_x_status = 0;     //加速度x值计数状态机
uint8 Acc_y_status = 0;     //加速度y值计数状态机
uint8 Acc_z_status = 0;     //加速度z值计数状态机

//重力四元素
float q0=1.0f;
float q1=0.0f;
float q2=0.0f;
float q3=0.0f;
//三个姿态角
float pitch=0.0;
float yaw=0.0;
float roll=0.0;
//
//float Kp=1.6f;
//float Ki=0.001f;

#define Kp_New      0.9f                            //互补滤波当前数据的权重
#define Kp_Old      0.1f                            //互补滤波历史数据的权重
#define Acc_Gain    0.0001220f                        //加速度变成G (初始化加速度满量程-+4g LSBa = 2*4/65535.0)
#define Gyro_Gain   0.0609756f                            //角速度变成度 (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)
#define Gyro_Gr     0.0010641f                            //角速度变成弧度(3.1415/180 * LSBg)


float   accb[3];                              //加速度计数据存储
float   gyrb[3];                               //角速度计数据存储
float   DCMgb[3][3];                          //方向余弦阵（将 惯性坐标系 转化为 机体坐标系）
uint8_t AccbUpdate = 0;                       //加速度计数据更新标志位
uint8_t GyrbUpdate = 0;                       //角速度计数据更新标志位
/*
 * @brief               陀螺仪处理初始化
 * @parameter device    使用的陀螺仪设备
 * @parameter time      使用的中断时间
 * @example             Gyroscope_Init(GYROSCOPE_IMU660RA, 10);
 */
void Gyroscope_Init(GYROSCOPE_TYPE device, uint16 time) {
    //陀螺仪偏移量初始量设置为0
    Gyro_Offset.ACC_Xdata = 0.0;
    Gyro_Offset.ACC_Ydata = 0.0;
    Gyro_Offset.ACC_Zdata = 0.0;
    Gyro_Offset.Gyro_Xdata = 0.0;
    Gyro_Offset.Gyro_Ydata = 0.0;
    Gyro_Offset.Gyro_Zdata = 0.0;

    //处理IMU660RA的代码
    if (device == GYROSCOPE_IMU660RA) {
        Gyroscope_device = GYROSCOPE_IMU660RA;
        //数据点循环采样150次,每采样一次,系统延时10ms,所以共需要延时1.5s
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
    //处理IMU963RA的代码
    else if (device == GYROSCOPE_IMU963RA) {
        Gyroscope_device = GYROSCOPE_IMU963RA;

    }
    //处理ICM20602的代码
    else if (device == GYROSCOPE_ICM20602) {
        Gyroscope_device = GYROSCOPE_ICM20602;
    }
    //传参数据先不做限制,自己传能用的数据过来~~~
    Gyroscope_time = time;
}

/*
 * @brief                   计时器开始计数
 * @parameter measureType   计数类型
 * @example                 Gyroscope_Begin(GYROSCOPE_GYRO_X);
 *
 */
void Gyroscope_Begin(GYROSCOPE_MEASURE_TYPE measureType) {
    //要求,只有计数器转态为0的时候,才能开始进行计数
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
 * @brief                   计数器停止计数
 * @parameter measureType   计数类型
 * @example                 Gyroscope_End(GYROSCOPE_GYRO_X);
 *
 */
void Gyroscope_End(GYROSCOPE_MEASURE_TYPE measureType) {
    //关掉的话就直接把状态置0了,不用考虑是否处于在工作状态
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
    //------------------------------处理数据------------------------------
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
    //------------------------------基本说明------------------------------
    //进行计数,放中断里面进行数据的累计
    //速度单位为m/s
    //角度单位为度
    //--------------------获取数据--------------------
    Gyroscope_GetData();
    //IMUupdate(imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z, imu660ra_acc_x,imu660ra_acc_y, imu660ra_acc_z, alpha, halfT);

    //--------------------数据处理--------------------
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
 * @brief                   清空积分的数据
 * @paramter measureType    测量的类型
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
    //MPU6050_Read();    //触发读取 ，立即返回
    //MPU6050_Offset();  //对MPU6050进行处理，减去零偏。如果没有计算零偏(根据标志位判断)就计算零偏
    Gyroscope_GetData();
    //MPU6050_AccRead(&Acc_filt);//获取加速度计原始数据，并进行卡尔曼滤波
    //MPU6050_GyroRead(&Gyr_filt);//获取角速度计原始数据，并进行一阶低通滤波

//  Aver_FilterXYZ(&Acc_filt,12);//此处对加速度计进行滑动窗口滤波处理
//  Aver_FilterXYZ(&Gyr_filt,12);//此处对角速度计进行滑动窗口滤波处理

    //加速度AD值 转换成 米/平方秒
    Acc_filtX = (float)Acc_corrX * Acc_Gain * G;
    Acc_filtY = (float)Acc_corrY * Acc_Gain * G;
    Acc_filtZ = (float)Acc_corrZ * Acc_Gain * G;
//  printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",Acc_filt.X,Acc_filt.Y,Acc_filt.Z);

    //陀螺仪AD值 转换成 弧度/秒
    Gyr_filtX = (float) Gyro_corrX * Gyro_Gr;
    Gyr_filtY = (float) Gyro_corrX * Gyro_Gr;
    Gyr_filtZ = (float) Gyro_corrX * Gyro_Gr;
//  printf("gx=%0.2f gy=%0.2f gz=%0.2f\r\n",Gyr_filt.X,Gyr_filt.Y,Gyr_filt.Z);
}

/*********************************************************************************************************
* 函  数：void IMUupdate(FLOAT_XYZ *Gyr_filt,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Att_Angle)
* 功　能：获取姿态角
* 参  数：Gyr_filt     指向角速度的指针（注意单位必须是弧度）
*         Acc_filt  指向加速度的指针
*         Att_Angle 指向姿态角的指针
* 返回值：无
* 备  注：求解四元数和欧拉角都在此函数中完成
**********************************************************************************************************/
//kp=ki=0 就是完全相信陀螺仪
#define Kp 1.5f                          // proportional gain governs rate of convergence to accelerometer/magnetometer
                                          //比例增益控制加速度计，磁力计的收敛速率
#define Ki 0.005f                         // integral gain governs rate of convergence of gyroscope biases
                                          //积分增益控制陀螺偏差的收敛速度
#define halfT 0.005f                      // half the sample period 采样周期的一半

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

    //加速度计<测量>的重力加速度向量(机体坐标系)
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
//  printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);

    //陀螺仪积分<估计>重力向量(机体坐标系)
    vx = 2*(q1q3 - q0q2);                  //矩阵(3,1)项
    vy = 2*(q0q1 + q2q3);                //矩阵(3,2)项
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;     //矩阵(3,3)项

    // printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz);

    //向量叉乘所得的值
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    //用上面求出误差进行积分
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    //将误差PI后补偿到陀螺仪
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

    //四元素的微分方程
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    //单位化四元数
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

//  矩阵表达式
//  matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;   // 11
//  matrix[1] = 2.f * (q1q2 + q0q3);           // 12
//  matrix[2] = 2.f * (q1q3 - q0q2);           // 13
//  matrix[3] = 2.f * (q1q2 - q0q3);           // 21
//  matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;   // 22
//  matrix[5] = 2.f * (q2q3 + q0q1);           // 23
//  matrix[6] = 2.f * (q1q3 + q0q2);           // 31
//  matrix[7] = 2.f * (q2q3 - q0q1);           // 32
//  matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;   // 33

    //四元数转换成欧拉角(Z->Y->X)

    //偏航角YAW
    if( (Gyr_filtZ *RadtoDeg > 1.0f) || (Gyr_filtZ *RadtoDeg < -1.0f) ) //数据太小可以认为是干扰，不是偏航动作
    {
        yaw += Gyr_filtZ *RadtoDeg*0.01f;
    }
//  printf("yaw: %f\r\n",Att_Angle->yaw);//YAW角数据（非常稳定）

    //横滚角ROLL
    roll = -asin(2.f * (q1q3 - q0q2))* 57.3f;
//  printf("%f\r\n",Att_Angle->rol);//ROLL角数据

    //俯仰角PITCH
    pitch = -atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f ;
//  printf("pitch: %f\r\n",Att_Angle->pit);//PITCH角数据
//    printf("%f,%f,%f\r\n",Att_Angle->pit,Att_Angle->rol,Att_Angle->yaw);//PITCH角数据

}

void imu_work(void){
    Prepare_Data();
    IMUupdate(Gyr_filtX, Gyr_filtY,Gyr_filtZ, Acc_filtX, Acc_filtY,Acc_filtZ);

}

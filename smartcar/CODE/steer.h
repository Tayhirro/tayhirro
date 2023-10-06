#ifndef __STEER_H__
#define __STEER_H__

#include "common.h"
#include "zf_pwm.h"


//为双舵机预留，摄像头组和单舵机可以置 0
#define STEER_DOUBLE 0


#define STEER_LEFT      2050     //舵机摆角左极限  这里的值是用于测试的，大家根据自己的舵机更改
#define STEER_MIDDLE    1880   //舵机摆角中值
#define STEER_RIGHT     1680   //舵机摆角右极限  


//#define  STEER_FTMN     FTM3
#define  STEER_FTMCH    PWMB_CH1_P74     //PWM5
#define  STEER_FREQ     125    //舵机频率   由于一个FTM模块只能输出一个频率，                               //所以两个舵机也只能用一个频率 这里统一100Hz
#define  STEER_INIT_VAL 1250   //舵机初始值，精度1/10000  即1200/10000  = 12%

void steer_init();

void PWMSetSteer(int angle_pwm);

void steerCtrl();

/*#if STEER_DOUBLE
    #define STEER_LEFT1      1350     //舵机摆角左极限
    #define STEER_MIDDLE1    1200     //舵机摆角中值
    #define STEER_RIGHT1     1050      //舵机摆角右极限
    
    
    //#define  STEER_FTMN     FTM1   //两个舵机共用
    #define  STEER_FTMCH1    FTM_CH0     //FTM1_CH0对应PTA13口
    //#define  STEER_FREQ     100    //舵机频率   由于一个FTM模块只能输出一个频率，                               //所以两个舵机也只能用一个频率 这里统一100Hz
    #define  STEER_INIT_VAL1 1200   //舵机初始值，精度1/10000  即1200/10000  = 12%
    void PWMSetSteer1(int angle_pwm);
    void steerCtrl1();
#endif*/
    
    
#endif
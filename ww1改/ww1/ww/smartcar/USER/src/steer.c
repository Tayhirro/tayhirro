#include "steer.h"

/********************************************************************
** 函数名称 : void steer_init() 
** 功能描述 : 设置舵机转向
** 入口     ：
** 出口     ：
** 说明     :  初始化PWM波及占空比
*********************************************************************/
 void steer_init()         
{
    //ASSERT_RST(STEER_INIT_VAL <= FTM_PRECISON,"舵机0最大值不能超过PWM精度值");        
    pwm_init(STEER_FTMCH,STEER_FREQ , STEER_MIDDLE);
/*#if STEER_DOUBLE
    //ASSERT_RST(STEER_INIT_VAL1 <= FTM_PRECISON,"舵机最大值不能超过PWM精度值");
    ftm_pwm_init(STEER_FTMN,STEER_FTMCH1,STEER_FREQ,STEER_INIT_VAL1); 
#endif*/
}

/********************************************************************
** 函数名称: void PWMSetSteer(int angle_pwm) 
** 功能描述: 设置舵机转向
** 入口：角度值
** 出口：
** 说明:  
*********************************************************************/
void PWMSetSteer(int angle_pwm)         
{
    //占空比不能超过上限（防止过压）
    //同时防止方向打死  
    //这个保护措施是必要的
    if(angle_pwm < STEER_RIGHT)
         angle_pwm = STEER_RIGHT;
    if(angle_pwm > STEER_LEFT)
         angle_pwm = STEER_LEFT;
        
    pwm_duty(STEER_FTMCH,angle_pwm);
}


/********************************************************************
** 函数名称: steerCtrl
** 功能描述: 速度控制算法
** 入口：
** 出口：
** 说明:  
*********************************************************************/
void steerCtrl()
{
    /*
    int angle_pwm;
    angle_pwm = 0;    //去除warning
    
    //自己的舵机控制算法
    
    
    PWMSetSteer(angle_pwm) ; 
    */ 
}



#ifndef __MOTOR_H__
#define __MOTOR_H__

#define MOTOR_DOUBLE 1   //为摄像头组两路控制预留

#include "common.h"
#include "zf_pwm.h"


#define MOTOR_MAX    9600
#define MOTOR_MIN    -9600


//#define  MOTOR_FTMN       FTM0            
#define  MOTOR_FTMCH0     PWMA_CH3P_P64          //PTC1
#define  MOTOR_FTMCH1     PWMA_CH4P_P66          //PTC2 
 

#define  MOTOR_FREQ 16000           //电机频率   16k
#define  MOTOR_INIT_VAL 0           //电机初始值，精度1/10000  即0/10000 0%


         void motor_init();

//#if !MOTOR_DOUBLE
  //  void PWMSetMotor(uint16 speed_pwm);
//#endif

//#if MOTOR_DOUBLE
    //#define MOTOR_MAX    10000
    //#define MOTOR_MIN    -10000
    
    //#define  MOTOR_FTMN       FTM0            
    #define  MOTOR_FTMCH2      PWMA_CH1P_P60            //PTC3
    #define  MOTOR_FTMCH3      PWMA_CH2P_P62           //PTC4 
    
    //#define  MOTOR_FREQ 16000           //电机频率   16k
    //#define  MOTOR_INIT_VAL 0           //电机初始值，精度1/10000  即0/10000 0%
        
    //void motor_init();

   
 extern   void PWMSetMotor2(int16 speed_pwmL,int16 speed_pwmR);
//#endif 

    
    void motorCtrl();
   void protect(void);   
#endif
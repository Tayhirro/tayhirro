#include "motor.h"
#include "zf_pwm.h"
#include "common.h"
#include "zf_delay.h"
#include "pidcc.h"
/********************************************************************
** º¯ÊýÃû³Æ: void motor_init() 
** ¹¦ÄÜÃèÊö: ÉèÖÃ¶æ»ú×ªÏò
** Èë¿Ú£º½Ç¶ÈÖµ
** ³ö¿Ú£º
** ËµÃ÷:  
*********************************************************************/
void motor_init()         
{    
    //³õÊ¼»¯FTM2_CH0Êä³öÆµÂÊÎª100HZ,Õ¼¿Õ±wÈÎª13%µÄPWM £ºFTM2_CH0¶ÔÓ¦PTA10¿Ú
    pwm_init( MOTOR_FTMCH0 ,MOTOR_FREQ, MOTOR_INIT_VAL); 
    pwm_init( MOTOR_FTMCH1 ,MOTOR_FREQ, MOTOR_INIT_VAL); //MOTOR_FREQ
//#if MOTOR_DOUBLE
    pwm_init( MOTOR_FTMCH2 ,MOTOR_FREQ, MOTOR_INIT_VAL); // MOTOR_INIT_VAL
    pwm_init( MOTOR_FTMCH3 ,MOTOR_FREQ, MOTOR_INIT_VAL); 
//#endif
}
//#if !MOTOR_DOUBLE
/********************************************************************
** º¯ÊýÃû³Æ: PWMSetMotor
** ¹¦ÄÜÃèÊö: ÉèÖÃËÙ¶È
** Èë¿Ú£ºËÙ¶È
** ³ö¿Ú£º
** ËµÃ÷:  
*********************************************************************/
/*void PWMSetMotor(uint16 speed_pwm)         
{
    if(speed_pwm < MOTOR_MIN)
    {
      speed_pwm = MOTOR_MIN;
    }
    if(speed_pwm > MOTOR_MAX)
    {
      speed_pwm = MOTOR_MAX;
    }
    
    if(speed_pwm > 0) 
    {
        pwm_duty(MOTOR_FTMCH0, (uint32)speed_pwm);
        pwm_duty(MOTOR_FTMCH1, 0);
    } 
    else 
    {
       speed_pwm = -speed_pwm;
       pwm_duty(MOTOR_FTMCH1, (uint32)speed_pwm);
        pwm_duty(MOTOR_FTMCH0, 0);
    }
}*/

/********************************************************************
** º¯ÊýÃû³Æ: motorCtrl
** ¹¦ÄÜÃèÊö: ËÙ¶È¿ØÖÆËã·¨
** Èë¿Ú£º
** ³ö¿Ú£º
** ËµÃ÷:  
*********************************************************************/
/*void motorCtrl()
{
    /***
  s32 speed_pwm;
    
    speed_pwm = 0;    //È¥³ýwarning
    
    //×Ô¼ºµÄËÙ¶È¿ØÖÆËã·¨
    
    
    
    PWMSetMotor(speed_pwm);
  ***/
//}*/

//#endif


//#if MOTOR_DOUBLE
/********************************************************************
** º¯ÊýÃû³Æ: PWMSetMotor
** ¹¦ÄÜÃèÊö: ÉèÖÃËÙ¶È
** Èë¿Ú£º×óÓÒËÙ¶È
** ³ö¿Ú£º
** ËµÃ÷:  
*********************************************************************/
void PWMSetMotor2(int16 speed_pwmL,int16 speed_pwmR)         
{
    if(speed_pwmL < MOTOR_MIN)
    {
      speed_pwmL = MOTOR_MIN;
    }
    if(speed_pwmL > MOTOR_MAX)
    {
      speed_pwmL = MOTOR_MAX;
    }
    
    if(speed_pwmR < MOTOR_MIN)
    {
      speed_pwmR = MOTOR_MIN;
    }
    if(speed_pwmR > MOTOR_MAX)
    {
      speed_pwmR = MOTOR_MAX;
    }
    
    if(speed_pwmL > 0) 
    {
	      pwm_duty(MOTOR_FTMCH0, (uint32)speed_pwmL);
			  pwm_duty(MOTOR_FTMCH1,100);
    } 
    else 
    {
       speed_pwmL = -speed_pwmL;
       
		    
			  
			  pwm_duty(MOTOR_FTMCH0, (uint32)speed_pwmL);
        pwm_duty(MOTOR_FTMCH1, 0);
    }
    
    if(speed_pwmR > 0) 
    {
        
		   pwm_duty(MOTOR_FTMCH2, (uint32)speed_pwmR);
       pwm_duty(MOTOR_FTMCH3, 100);
    } 
    else 
    {
       speed_pwmR = -speed_pwmR;
			 pwm_duty(MOTOR_FTMCH2, (uint32)speed_pwmR);
       pwm_duty(MOTOR_FTMCH3,0);
    }
}

/********************************************************************
** º¯ÊýÃû³Æ: motorCtrl
** ¹¦ÄÜÃèÊö: ËÙ¶È¿ØÖÆËã·¨
** Èë¿Ú£º
** ³ö¿Ú£º
** ËµÃ÷:  
*********************************************************************/
void motorCtrl()
{
  
    /*s32 speed_pwm;
    
    speed_pwm = 0;    //È¥³ýwarning
    
    //×Ô¼ºµÄËÙ¶È¿ØÖÆËã·¨
    
    
    
    PWMSetMotor2(speed_pwm,speed_pwm);
  */
}
extern protect_motor;
extern adc_left_mid;
extern adc_right_mid;

void protect(void){
  if(adc_left_mid<30 && adc_right_mid<30){
             protect_motor=0;}
}
//#endif




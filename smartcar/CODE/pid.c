#include "pid.h"
#include "common.h"
#include "headfile.h"
#include "motor.h"
#include "steer.h"
int begin_speed = 3000;//初始速度
int lasterror = 0;//上次误差
int accerror = 0;//累积误差(intergral)
//方向环pid

//#define test_port UART_0
//#define NUMPIX 188
///extern uint16 bigbendflag;


float in1,out1,in2,out2,lastin1=0,lastin2=0;
float setpoint1=3000.0;
float setpoint2=3000.0;
float pp1=100.0;
float pp2=100.0;
float ii1=0.0;
float ii2=0.0;
float dd1=1;
float dd2=1;
float max_speed=7000;
float min_speed=-7000;
float iterm1=0;
float iterm2=0;

/*void speed_dynamic()
{

}*/

//spead control 
/* void pid_compute_new(float in1,float in2)
{ 
    float error1;
	float error2;
	float out1,out2;
	error1=setpoint1-in1;
    iterm1+=error1;
    
    error2=setpoint2-in2;
    iterm2+=error2;
    
    if(error1 < 0)
    {
      p1 = 300;
    }
    else
    {
      p1 = 100;
    }
    
    if(error2 < 0)
    {
      p2 = 300;
    }
    else
    {
      p2 = 100;
    }
    
    if(iterm1>max_speed)
      iterm1=max_speed;
    else if(iterm1<min_speed)
      iterm1=min_speed;
    if(iterm2>max_speed)
      iterm2=max_speed;
    else if(iterm2<min_speed)
      iterm2=min_speed;
    
     out1=pp1*error1+ii1*iterm1-dd1*(in1-lastin1);
     out2=pp2*error2+ii2*iterm2-dd2*(in2-lastin2);
    
    if(out1>max_speed)
      out1=max_speed;
    else if(out1<min_speed)
      out1=min_speed;
    if(out2>max_speed)
      out2=max_speed;
    else if(out2<min_speed)
      out2=min_speed;
    
    lastin1=in1;
    lastin2=in2;
   // uart_printf(UART_0,"out = %f %f \n",out1,out2);
    
    PWMSetMotor2(out1,out2); 
}


      

      
    
//电磁控制参数
    

//#define  AD_CH0                ADC0_SE16
//#define  AD_CH1                ADC1_SE18
#define  g_HighestSpeed        100                      //编码器读回的定值
#define  g_LowestSpeed         0                       //编码器读回的定值


float  dir_control_P=0;                     // 4         2.7         7  
float  dir_control_D=0;          // 155        140        100
  
float     ADresult0=0,ADresult1=0;
int var[2];

int     dir_error=0,last_dir_error=0;
float   dir_P_value=0,dir_D_value=0;
float   g_fDirectionControlOut=0;

int     flag=0,lastflag=0,time=0;

int min(int a,int b)
{
  return a<b?a:b;
}

//方向环PID
    
void ele_direction_control()
{
        int i = H-21;
        ADresult0 = 0;
        
        for(;i>H-41;i--)
        {
          ADresult0 += Pick_table[i];     
        }
        
        ADresult1 = NUMPIX*10;
        dir_error = (int)(ADresult0-ADresult1)/20;
        
	 
        //uart_printf(UART_0,"dir_error =  %d\n",dir_error);

                       
        if(-30>dir_error)
        {
            dir_control_P=5;
            dir_P_value=dir_control_P*dir_error;
            if(speedflag == 1)
            {
                setpoint1 = NORMAL_SPEED+2.5*dir_error;
                setpoint2 = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
                setpoint1 = CIRCLE_SPEED+2*dir_error;
                setpoint2 = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint2 = STOP_SPEED;
                setpoint1 = STOP_SPEED+1*dir_error;
            }
        }
        else if(-15>=dir_error && dir_error>=-30)
        {
            dir_control_P=5;
            dir_P_value=dir_control_P*dir_error;
            if(speedflag == 1)
            {
              setpoint1 = NORMAL_SPEED+1*dir_error;
              setpoint2 = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
                setpoint1 = CIRCLE_SPEED+1*dir_error;
                setpoint2 = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint2 = STOP_SPEED;
                setpoint1 = STOP_SPEED+1*dir_error;
            }
        }
        else if(-5>dir_error && dir_error>-15)
        {
            dir_control_P=5;
            dir_P_value=dir_control_P*dir_error;
            if(speedflag == 1)
            {
              setpoint1 = NORMAL_SPEED+1*dir_error;
              setpoint2 = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
                setpoint1 = CIRCLE_SPEED+1*dir_error;
                setpoint2 = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint2 = STOP_SPEED;
                setpoint1 = STOP_SPEED+1*dir_error;
            }
        } 
        else if(-5<=dir_error && dir_error<0)
        {
            dir_control_P=4;
            dir_P_value=dir_control_P*dir_error;
           if(speedflag == 1)
            {
              setpoint1 = setpoint2 = NORMAL_SPEED;
            }
              else if(speedflag == 2)
            {
              setpoint1 = setpoint2 = CIRCLE_SPEED;
            }
           
        }
        else if(0<=dir_error && dir_error<=5)
        {
            dir_control_P=4;
            dir_P_value=dir_control_P*dir_error;
           if(speedflag == 1)
            {
              setpoint1 = setpoint2 = NORMAL_SPEED;
            }
           else if(speedflag == 2)
            {
              setpoint1 = setpoint2 = CIRCLE_SPEED;
            }
           
        }
        else if(5<dir_error && dir_error<15)
        {
            dir_control_P=5;
            dir_P_value=dir_control_P*dir_error;
           if(speedflag == 1)
            {
              setpoint2 = NORMAL_SPEED-1*dir_error;
                setpoint1 = NORMAL_SPEED;
            }
           else if(speedflag == 2)
            {
              setpoint2 = CIRCLE_SPEED-1*dir_error;
                setpoint1 = CIRCLE_SPEED;
            }
           else if(speedflag == 5)
            {
              setpoint2 = STOP_SPEED-1*dir_error;
                setpoint1 = STOP_SPEED;
            }
        }
        else if(15<=dir_error && dir_error<30)
        {
            dir_control_P=5;
            dir_P_value=dir_control_P*dir_error;
            if(speedflag == 1)
            {
              setpoint2 = NORMAL_SPEED-1*dir_error;
              setpoint1 = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
              setpoint2 = CIRCLE_SPEED-1*dir_error;
                setpoint1 = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint2 = STOP_SPEED-1*dir_error;
                setpoint1 = STOP_SPEED;
            }
        }else if(30<=dir_error)
        {
            dir_control_P=5;
            dir_P_value=dir_control_P*dir_error;
            if(speedflag == 1)
            {
              setpoint2 = NORMAL_SPEED-2.5*dir_error;
              setpoint1 = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
              setpoint2 = CIRCLE_SPEED-2*dir_error;
              setpoint1 = CIRCLE_SPEED;
            }else if(speedflag == 5)
            {
              setpoint2 = STOP_SPEED-1*dir_error;
                setpoint1 = STOP_SPEED;
            }
        }
        
        
        if(dir_P_value<-160)dir_P_value=-160;
        if(dir_P_value>160)dir_P_value=160;
        
        //uart_printf(UART_0,"output = %f\n",dir_P_value);
        
        dir_control_D=0.1;
        dir_D_value=dir_control_D*(dir_error-last_dir_error);
        
        g_fDirectionControlOut=dir_P_value+dir_D_value; 
        last_dir_error=dir_error;

        
        pwm_duty(,(1250-g_fDirectionControlOut));
}
*/

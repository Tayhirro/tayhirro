#include "pidcc.h"
#include "common.h"
#include "headfile.h"
#include "motor.h"
#include "steer.h"
#include "encoder.h"

#define LOC_INTEGRAL_START_ERR 200  /*积分分离时对应的误差范围*/
#define LOC_INTEGRAL_MAX_VAL 800   /*积分范围限定，防止积分饱和*/
#define SPE_DEAD_ZONE   5.0f       /*速度环死区*/

// steer pid

#define con 20             //adc转换常数           （可调）
#define NORMAL_SPEED 1800  //直道速度							 （可调）
#define STOP_SPEED 1500    //检测到停止标志时的速度（可调）
#define CIRCLE_SPEED 1600  //进环速度              （可调）

#define angle_left_pwm    1980     //左环岛 入环舵机打角对应pwm                                                （可调）
#define angle_right_pwm   1680     //右环岛 入环舵机打角对应pwm 																							 （可调）
#define incircle_delay_ms   20     //不用编码器计算路程的话，该值为检测到预进环标志后，打固定角之前延时的时长   (可调)
#define outcircle_delay_ms  20     //不用编码器计算路程的话，该值为检测到预出环标志后，打固定角之后延时的时长   (可调)

#define incircle_threshold1  10    //入环最左adc减最右adc判定阈值                                              （可调）
#define incircle_threshold2  10    //入环中间adc判定阈值                                                       （可调）
//#define incircle_threshold3  10    //入环adc判定阈值     需要时可再加
#define outcircle_threshold1  10    //出环最左adc减最右adc判定阈值                                              （可调）
#define outcircle_threshold2  10    //出环中间adc判定阈值                                                       （可调）
//#define outcircle_threshold3  10    //出环adc判定阈值     需要时可再加


int speedflag=1;           //直道、停止、进环判断标志
PID pid_steer;
iPID ipid_speed_left;
iPID ipid_speed_right;
toiPID toipid_speed_left=&ipid_speed_left;
toiPID toipid_speed_right=&ipid_speed_right;

float setpoint_left=1800.0; // setpoint_left -> left 左边标准速度    setpoint_right -> right  右边标准速度
float setpoint_right=1800.0;
int adc_left_edge,  adc_left_mid,  adc_mid,  adc_right_mid,  adc_right_edge;  //最左、中左、中、中右、最右adc的读取数值（应设置为全局变量）
int status=1;                      //出入环标志位(环岛标志位)，检测到预入环状态后，值为0，否则为1,即一般情况下值都为1


void PID_param_init()
{
    /* ????????? */
    pid_steer.target_val =0;               
    pid_steer.output_val = 1880;
    pid_steer.err = 0.0;
    pid_steer.err_last = 0.0;
    pid_steer.integral = 0.0;

    pid_steer.Kp = 0.05;
    pid_steer.Ki = 0;
    pid_steer.Kd = 0;
    pid_steer.input_val=0;
	  pid_steer.Kp_output_val=0;
	  pid_steer.Ki_output_val=0;
	  pid_steer.Kd_output_val=0;
    /* ????????? */
    ipid_speed_left.target_val=10.0;              
    ipid_speed_left.output_val=0.0;
    ipid_speed_left.err=0.0;
    ipid_speed_left.err_last=0.0;
	
    ipid_speed_left.Kp_output_val=0;
		ipid_speed_left.Ki_output_val=0;
		ipid_speed_left.Kd_output_val=0;
		
    ipid_speed_left.Kp = 80.0;
    ipid_speed_left.Ki = 2.0;
    ipid_speed_left.Kd = 100.0;
		ipid_speed_left.input_val=0;
		ipid_speed_left.integral=0.0;
		
		ipid_speed_right.target_val=10.0;              
    ipid_speed_right.output_val=0.0;
    ipid_speed_right.err=0.0;
    ipid_speed_right.err_last=0.0;

		ipid_speed_right.Kp_output_val=0;
		ipid_speed_right.Ki_output_val=0;
		ipid_speed_right.Kd_output_val=0;
		
		ipid_speed_right.integral=0.0;

    ipid_speed_right.Kp = 80.0;
    ipid_speed_right.Ki = 2.0;
    ipid_speed_right.Kd = 100.0;
		ipid_speed_right.input_val=0;
		
		
		ipid_speed_left.omin=4000;
		ipid_speed_left.omax=4000;
		ipid_speed_right.omin=4000;
		ipid_speed_right.omax=4000;
}

//**************************//
//*****环岛入环adc检测******//
//**************************//
int judge_circle_in(float adc_left_edge,float adc_right_edge,float adc_mid){
        if(adc_left_edge - adc_right_edge > incircle_threshold1 && adc_mid > incircle_threshold2)return 1;
	      else return 0;
}//**************************//
//*****环岛出环adc检测******//
//**************************//
int judge_circle_out(float adc_left_edge,float adc_right_edge,float adc_mid){
        if(adc_left_edge - adc_right_edge > outcircle_threshold1 && adc_mid > outcircle_threshold2)return 1;
	      else return 0;
}
//***********************//
//*****环岛检测函数******//
//***********************//
//环岛检测放主函数里       停车标志位到时候放外部中断
void judge_circle(){
    if(status  &&  judge_circle_in(adc_left_edge,adc_right_edge,adc_mid) ){
		     status=0;
			   delay_ms(incircle_delay_ms);     //直道速度快，冲进环岛延时设置需要小一点
			   PWMSetSteer(angle_left_pwm);
			   speedflag=2;
			   
		}
		
		if(status == 0 && judge_circle_out(adc_left_mid,adc_right_mid,adc_mid) ){
		     speedflag=1;
			   PWMSetSteer(angle_left_pwm);
			   delay_ms(outcircle_delay_ms);  
			   status=1;	
		} 
//		else if(judge_right(adc_right_edge,adc_right_mid,adc_mid)=1){
//		     circleflag=1;
//			   PWMSetSteer(angle_right_pwm);
//			   speedflag=2;
//		}

}

//***************************//
//*****adc采集加舵机pid******//
//***************************//
void pid_steers(){
	adc_left_mid=Filter(ADC_P01,ADC_12BIT);
	adc_right_mid=Filter(ADC_P00,ADC_12BIT);
	//adc_left_edge=Filter(ADC_P00,ADC_12BIT); //改通道
	//adc_right_edge=Filter(ADC_P00,ADC_12BIT);//改通道
	//adc_mid=Filter(ADC_P00,ADC_12BIT);       //改通道
	ele_direction_control(&pid_steer,&ipid_speed_left,&ipid_speed_right,adc_left_mid,adc_right_mid);
}

void ele_direction_control(PID* topid_steer,iPID* toipid_speed_left,iPID* toipid_speed_right,float adc_left_mid,float adc_right_mid)
{
        //int i = H-21;
        //ADresult0 = 0;
        
        //for(;i>H-41;i--)
        //{
        // ADresult0 += Pick_table[i];     
        //}
        
        //ADresult1 = NUMPIX*10;
        // topid_steer->err = (int)(ADresult0-ADresult1)/20;
        topid_steer->err =(int)(adc_left_mid-adc_right_mid)/con;//计算差值，左减右
	 
        //uart_printf(UART_0," topid_steer->err =  %d\n", topid_steer->err);

                       
        if(-35>topid_steer->err)//右偏过大
        {
            topid_steer->Kp=5;
            topid_steer->Kp_output_val=topid_steer->Kp* topid_steer->err;
            if(speedflag == 1)
            {
                setpoint_left = NORMAL_SPEED+2.5*topid_steer->err;
                setpoint_right = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
                setpoint_left = CIRCLE_SPEED+2*topid_steer->err;
                setpoint_right = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint_right = STOP_SPEED;
                setpoint_left = STOP_SPEED+1*topid_steer->err;
            }
        }
				
        else if(-20>=topid_steer->err && topid_steer->err>=-35)//右偏较大
        {
            topid_steer->Kp=6;
            topid_steer->Kp_output_val=topid_steer->Kp*topid_steer->err;
            if(speedflag == 1)
            {
              setpoint_left = NORMAL_SPEED+1*topid_steer->err;
              setpoint_right = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
                setpoint_left = CIRCLE_SPEED+1*topid_steer->err;
                setpoint_right = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint_right = STOP_SPEED;
                setpoint_left = STOP_SPEED+1*topid_steer->err;
            }
        }
        else if(-10>topid_steer->err && topid_steer->err>-20)//右偏较小
        {
            topid_steer->Kp=2;
            topid_steer->Kp_output_val=topid_steer->Kp*topid_steer->err;
            if(speedflag == 1)
            {
              setpoint_left = NORMAL_SPEED+1*topid_steer->err;
              setpoint_right = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
                setpoint_left = CIRCLE_SPEED+1*topid_steer->err;
                setpoint_right = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint_right = STOP_SPEED;
                setpoint_left = STOP_SPEED+1*topid_steer->err;
            }
        } 
        else if(-10<=topid_steer->err && topid_steer->err<0)//基本无右偏
        {
            topid_steer->Kp=1;
            topid_steer->Kp_output_val=topid_steer->Kp*topid_steer->err;
           if(speedflag == 1)
            {
              setpoint_left = setpoint_right = NORMAL_SPEED;
            }
              else if(speedflag == 2)
            {
              setpoint_left = setpoint_right = CIRCLE_SPEED;
            }
           
        }
        else if(0<=topid_steer->err && topid_steer->err<=10)//基本无左偏
        {
            topid_steer->Kp=1;
            topid_steer->Kp_output_val=topid_steer->Kp*topid_steer->err;
           if(speedflag == 1)
            {
              setpoint_left = setpoint_right = NORMAL_SPEED;
            }
           else if(speedflag == 2)
            {
              setpoint_left = setpoint_right = CIRCLE_SPEED;
            }
           
        }
        else if(10<topid_steer->err && topid_steer->err<20)//左偏较小
        {
            topid_steer->Kp=2;
            topid_steer->Kp_output_val=topid_steer->Kp*topid_steer->err;
           if(speedflag == 1)
            {
              setpoint_right = NORMAL_SPEED-1*topid_steer->err;
                setpoint_left = NORMAL_SPEED;
            }
           else if(speedflag == 2)
            {
              setpoint_right = CIRCLE_SPEED-1*topid_steer->err;
                setpoint_left = CIRCLE_SPEED;
            }
           else if(speedflag == 5)
            {
              setpoint_right = STOP_SPEED-1*topid_steer->err;
                setpoint_left = STOP_SPEED;
            }
        }
        else if(20<=topid_steer->err && topid_steer->err<35)//左偏较大
        {
            topid_steer->Kp=6;
            topid_steer->Kp_output_val=topid_steer->Kp*topid_steer->err;
            if(speedflag == 1)
            {
              setpoint_right = NORMAL_SPEED-1*topid_steer->err;
              setpoint_left = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
              setpoint_right = CIRCLE_SPEED-1*topid_steer->err;
                setpoint_left = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint_right = STOP_SPEED-1*topid_steer->err;
                setpoint_left = STOP_SPEED;
            }
        }else if(35<=topid_steer->err)//左偏过大
        {
            topid_steer->Kp=5;
            topid_steer->Kp_output_val=topid_steer->Kp*topid_steer->err;
            if(speedflag == 1)
            {
              setpoint_right = NORMAL_SPEED-2.5*topid_steer->err;
              setpoint_left = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
              setpoint_right = CIRCLE_SPEED-2*topid_steer->err;
              setpoint_left = CIRCLE_SPEED;
            }else if(speedflag == 5)
            {
                setpoint_right = STOP_SPEED-1*topid_steer->err;
                setpoint_left = STOP_SPEED;
            }
        }
        
        
        if(topid_steer->Kp_output_val<-200)topid_steer->Kp_output_val=-200;//对Kp觉得的输入值进行限幅
        if(topid_steer->Kp_output_val>200)topid_steer->Kp_output_val=200;  //对Kp觉得的输入值进行限幅
        
        //uart_printf(UART_0,"output = %f\n",dir_P_value);
        
        topid_steer->Kd=15;                                              //赋值Kd
        topid_steer->Kd_output_val=topid_steer->Kd*(topid_steer->err-topid_steer->err_last); //计算关于Kd的输入值
          
        topid_steer->output_val=(int)(topid_steer->Kp_output_val+topid_steer->Kd_output_val);   //Kd输入值+Kp输入值  构成最终的输入值
        topid_steer->err_last=topid_steer->err;                               //将此次计算的偏差保存到last_err里，以备微分的运算

				
				
				toipid_speed_left->target_val=(int)setpoint_left;																															//传给速度环
				toipid_speed_right->target_val=	(int)setpoint_right;	
				
        PWMSetSteer(1880+topid_steer->output_val);                        //输出舵机
}

//速度环   传入目标速度 
int speedleft_pid_realize(iPID *toipid_left_speed ,int actual_val_left ){
  int error_left;
	error_left=toipid_left_speed->target_val-actual_val_left;
	toipid_left_speed->err=error_left;
	
	if(error_left < 0)
    {
      toipid_left_speed->Kp = 300;
    }
    else
    {
      toipid_left_speed->Kp = 100;
    }
    
		
		   //死区控制
		    if( (toipid_left_speed->err>-SPE_DEAD_ZONE) && (toipid_left_speed->err<SPE_DEAD_ZONE ) )
    {
        toipid_left_speed->err = 0;
        toipid_left_speed->integral = 0;
        toipid_left_speed->err_last = 0;
    }


				//积分分离
		if(toipid_left_speed->err> -LOC_INTEGRAL_START_ERR && toipid_left_speed->err < LOC_INTEGRAL_START_ERR){
	      toipid_left_speed->integral += toipid_left_speed->err;  
        /*积分范围限定，防止积分饱和*/
        if(toipid_left_speed->integral > LOC_INTEGRAL_MAX_VAL)
        {
           toipid_left_speed->integral = LOC_INTEGRAL_MAX_VAL;
        }
        else if(toipid_left_speed->integral < -LOC_INTEGRAL_MAX_VAL)
        {
            toipid_left_speed->integral= -LOC_INTEGRAL_MAX_VAL;
        }
			}
				
			//速度限幅
		 if(toipid_left_speed->integral>toipid_left_speed->omax)
      toipid_left_speed->integral=toipid_left_speed->omax;
    else if(toipid_left_speed->integral<toipid_left_speed->omin)
      toipid_left_speed->integral=toipid_left_speed->omin;
		
		//输出
		toipid_left_speed->output_val=toipid_left_speed->Kp*toipid_left_speed->err+toipid_left_speed->Ki*toipid_left_speed->integral+toipid_left_speed->Kd*(toipid_left_speed->err-toipid_left_speed->err_last);
		toipid_left_speed->err_last= toipid_left_speed->err;
		return (int)toipid_left_speed->output_val;

}
int speedright_pid_realize(iPID *toipid_right_speed,int actual_val_right){
	int error_right;
	error_right=toipid_right_speed->target_val-actual_val_right;
	toipid_right_speed->err=error_right;
	
    if(error_right< 0)
    {
     toipid_right_speed->Kp = 300;
    }
    else
    {
     toipid_right_speed->Kp= 100;
    }
		
		   //死区控制
		    if( (toipid_right_speed->err>-SPE_DEAD_ZONE) && (toipid_right_speed->err<SPE_DEAD_ZONE ) )
    {
        toipid_right_speed->err = 0;
        toipid_right_speed->integral = 0;
        toipid_right_speed->err_last = 0;
    }


				//积分分离
			if(toipid_right_speed->err> -LOC_INTEGRAL_START_ERR && toipid_right_speed->err < LOC_INTEGRAL_START_ERR){
	      toipid_right_speed->integral += toipid_right_speed->err;  
        /*积分范围限定，防止积分饱和*/
        if(toipid_right_speed->integral > LOC_INTEGRAL_MAX_VAL)
        {
           toipid_right_speed->integral = LOC_INTEGRAL_MAX_VAL;
        }
        else if(toipid_right_speed->integral < -LOC_INTEGRAL_MAX_VAL)
        {
            toipid_right_speed->integral= -LOC_INTEGRAL_MAX_VAL;
        }
		}
				
			//速度限幅
    if(toipid_right_speed->integral>toipid_right_speed->omax)
      toipid_right_speed->integral=toipid_right_speed->omax;
    else if(toipid_right_speed->integral<toipid_right_speed->omin)
      toipid_right_speed->integral=toipid_right_speed->omin;
		
		//输出
		toipid_right_speed->output_val=toipid_right_speed->Kp*toipid_right_speed->err+toipid_right_speed->Ki*toipid_right_speed->integral+toipid_right_speed->Kd*(toipid_right_speed->err-toipid_right_speed->err_last);
    toipid_right_speed->err_last= toipid_right_speed->err;
		return (int)toipid_right_speed->output_val;
}
void  speed_pid(){
    PWMSetMotor2(speedleft_pid_realize(&ipid_speed_left,ipid_speed_left.input_val),speedright_pid_realize(&ipid_speed_right,ipid_speed_right.input_val));
}
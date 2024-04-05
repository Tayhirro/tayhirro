/*
 * xiao_(*PID).c
 *
 *  Created on: 2024��2��28��
 *      Author: ��
 */
#include "xiao_pid.h"
#include "xiao_steer.h"
#include "xiao_trace.h"
float NORMAL_SPEED=96;
float STOP_SPEED=0;
float CIRCLE_SPEED=40;
//float Motor_1Target=80; // Motor_1Target -> left ��߱�׼�ٶ�    Motor_2Target -> right  �ұ߱�׼�ٶ�
//float Motor_2Target=80;
int speedflag=1;           //ֱ����ֹͣ�������жϱ�־
fPID Trace_cameraLeftPID={0};                       //����߻�ȡ�����ߵ�PID
fPID Trace_cameraRightPID={0};                      //�ұ��߻�ȡ�����ߵ�PID
fPID Trace_cameraMidPID={0};                       //��+�һ�ȡ�����ߵ�PID

//iPID ipid_speed_left;
//iPID ipid_speed_right;
PID_TYPE pid_type=PID_INV;
//���PID
fPID Motor_1PID;
fPID Motor_2PID;

void PID_Init(fPID* PID){
    (*PID).Kp=0;
    (*PID).Ki=0;
    (*PID).Kd=0;
    (*PID).ut=0;

    (*PID).Kp_Set=0;
    (*PID).Ki_Set=0;
    (*PID).Kd_Set=0;
    (*PID).target_val=0;
    (*PID).Kp_output_val=0;
    (*PID).Ki_output_val=0;
    (*PID).Kd_output_val=0;
    (*PID).output_val=0;
    (*PID).input_val=0;
    (*PID).err=0;
    (*PID).err_last=0;
    (*PID).err_llast=0;
    (*PID).integral=0;

    (*PID).sumLimit=0;
    (*PID).utLimit=0;
    (*PID).boost=0;
}

/*void fPID_Init(fPID* PID){
    PID->Kp=0;
    PID->Ki=0;
    PID->Kd=0;

    PID->Kp_Set=0;
    PID->Ki_Set=0;
    PID->Kd_Set=0;

    PID->target_val=0;
    PID->Kp_output_val=0;
    PID->Ki_output_val=0;
    PID->Kd_output_val=0;
    PID->output_val=0;
    PID->input_val=0;

    PID->ut=0;
    PID->err=0;

    PID->err_last=0;
    PID->err_llast=0;
    PID->integral=0;

    PID->sumLimit=0;
    PID->utLimit=0;
}*/

void Steer_PID_Init(void){
    PID_Init(&Trace_cameraLeftPID);
    PID_Init(&Trace_cameraMidPID);
    PID_Init(&Trace_cameraRightPID);
}
void direction_control(fPID* topid_steer,float zhongxian,float target)
{
        //(*topid_steer).Kp=(*topid_steer).Kp_Set;
        //ips200_show_float(100, 250, (*topid_steer).Kp, 3, 3);
        (*topid_steer).err =(int)(zhongxian-target)*2;//�����ֵ�������
        //uart_printf(UART_0," topid_steer->err =  %d\n", topid_steer->err);
if(Trace_Status==TRACE_CROSS || Trace_Status==TRACE_CIRCLE_RIGHT || TRACE_CIRCLE_LEFT){speedflag =2;}

        if(-30>(*topid_steer).err)//��ƫ����
        {if(Trace_Status==TRACE_CENTERLINENEAR){speedflag =2;}
        (*topid_steer).Kd=1.0;
            (*topid_steer).Kp=1.0;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp* (*topid_steer).err;
            if(speedflag == 1)
            {
                Motor_2Target = NORMAL_SPEED-0.5*(*topid_steer).err;
                Motor_1Target = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
                Motor_2Target = CIRCLE_SPEED-1.0*(*topid_steer).err;
                Motor_1Target = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              Motor_1Target = STOP_SPEED;
                Motor_2Target = STOP_SPEED-0.5*(*topid_steer).err;
            }
        }

        else if(-18>=(*topid_steer).err && (*topid_steer).err>=-30)//��ƫ�ϴ�
        {if(Trace_Status==TRACE_CENTERLINENEAR){speedflag =2;}
        (*topid_steer).Kd=0.7;
            (*topid_steer).Kp=0.8;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
            if(speedflag == 1)
            {
              Motor_2Target = NORMAL_SPEED-0.5*(*topid_steer).err;
              Motor_1Target = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
                Motor_2Target = CIRCLE_SPEED-1.0*(*topid_steer).err;
                Motor_1Target = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              Motor_1Target = STOP_SPEED;
                Motor_2Target = STOP_SPEED-0.5*(*topid_steer).err;
            }
        }
        else if(-13>(*topid_steer).err && (*topid_steer).err>-18)//��ƫ��С
        {if(Trace_Status==TRACE_CENTERLINENEAR){speedflag =1;}
            (*topid_steer).Kp=0.6;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
            if(speedflag == 1)
            {
              Motor_2Target = NORMAL_SPEED-0.1*(*topid_steer).err;
              Motor_1Target =NORMAL_SPEED ;
            }
            else if(speedflag == 2)
            {
                Motor_2Target = CIRCLE_SPEED-0.2*(*topid_steer).err;
                Motor_1Target = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              Motor_1Target = STOP_SPEED;
                Motor_2Target = STOP_SPEED-0.5*(*topid_steer).err;
            }
        }
        else if(-13<=(*topid_steer).err && (*topid_steer).err<0)//��������ƫ
        {if(Trace_Status==TRACE_CENTERLINENEAR){speedflag =1;}
            (*topid_steer).Kp=0.5;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
           if(speedflag == 1)
            {
              Motor_1Target =NORMAL_SPEED;
              Motor_2Target =NORMAL_SPEED ;
            }
              else if(speedflag == 2)
            {
              Motor_1Target = Motor_2Target = CIRCLE_SPEED;
            }

        }
        else if(0<=(*topid_steer).err && (*topid_steer).err<=13)//��������ƫ
        {if(Trace_Status==TRACE_CENTERLINENEAR){speedflag =1;}
            (*topid_steer).Kp=0.5;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
           if(speedflag == 1)
            {
              Motor_1Target =NORMAL_SPEED;
              Motor_2Target =NORMAL_SPEED ;
            }
           else if(speedflag == 2)
            {
              Motor_1Target = Motor_2Target = CIRCLE_SPEED;
            }

        }
        else if(13<(*topid_steer).err && (*topid_steer).err<18)//��ƫ��С
        {if(Trace_Status==TRACE_CENTERLINENEAR){speedflag =1;}
            (*topid_steer).Kp=0.6;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
           if(speedflag == 1)
            {
              Motor_1Target =NORMAL_SPEED +0.1*(*topid_steer).err;
                Motor_2Target =NORMAL_SPEED ;
            }
           else if(speedflag == 2)
            {
              Motor_1Target = CIRCLE_SPEED+0.2*(*topid_steer).err;
                Motor_2Target = CIRCLE_SPEED;
            }
           else if(speedflag == 5)
            {
              Motor_1Target = STOP_SPEED+1*(*topid_steer).err;
                Motor_2Target = STOP_SPEED;
            }
        }
        else if(18<=(*topid_steer).err && (*topid_steer).err<30)//��ƫ�ϴ�
        {if(Trace_Status==TRACE_CENTERLINENEAR){speedflag =2;}
        (*topid_steer).Kd=0.7;
            (*topid_steer).Kp=0.8;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
            if(speedflag == 1)
            {
              Motor_1Target =NORMAL_SPEED +0.5*(*topid_steer).err;
              Motor_2Target =NORMAL_SPEED ;
            }
            else if(speedflag == 2)
            {
              Motor_1Target = CIRCLE_SPEED+1.0*(*topid_steer).err;
                Motor_2Target = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              Motor_1Target = STOP_SPEED+0.5*(*topid_steer).err;
                Motor_2Target = STOP_SPEED;
            }
        }else if(30<=(*topid_steer).err)//��ƫ����
        {if(Trace_Status==TRACE_CENTERLINENEAR){speedflag =2;}
        (*topid_steer).Kd=1.0;
            (*topid_steer).Kp=1.0;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
            if(speedflag == 1)
            {
              Motor_1Target =NORMAL_SPEED +0.5*(*topid_steer).err;
              Motor_2Target =NORMAL_SPEED ;
            }
            else if(speedflag == 2)
            {
              Motor_1Target = CIRCLE_SPEED+1.0*(*topid_steer).err;
              Motor_2Target = CIRCLE_SPEED;
            }else if(speedflag == 5)
            {
                Motor_1Target = STOP_SPEED+0.5*(*topid_steer).err;
                Motor_2Target = STOP_SPEED;
            }
        }


        if((*topid_steer).Kp_output_val<-(*topid_steer).pLimit)(*topid_steer).Kp_output_val=-(*topid_steer).pLimit;//��Kp���õ�����ֵ�����޷�
        if((*topid_steer).Kp_output_val>(*topid_steer).pLimit)(*topid_steer).Kp_output_val=(*topid_steer).pLimit;  //��Kp���õ�����ֵ�����޷�

        //uart_printf(UART_0,"output = %f\n",dir_P_value);

        //(*topid_steer).Kd=(*topid_steer).Kd_Set;                                              //��ֵKd
        (*topid_steer).Kd_output_val=(*topid_steer).Kd*((*topid_steer).err-(*topid_steer).err_last); //�������Kd������ֵ

        (*topid_steer).output_val=(*topid_steer).boost*((*topid_steer).Kp_output_val+(*topid_steer).Kd_output_val);   //Kd����ֵ+Kp����ֵ  �������յ�����ֵ

        (*topid_steer).err_last=(*topid_steer).err;                               //���˴μ����ƫ��浽last_err��Ա�΢�ֵ�����



              //  toipid_speed_left.target_val=(int)Motor_1Target;                                                                                                                           //�����ٶȻ�
              //  toipid_speed_right.target_val= (int)Motor_2Target;
        //ips200_show_float(100, 250, (*topid_steer).output_val, 3, 3);
        PWMSetSteer(90.0-(*topid_steer).output_val);                        //������
}

void PID_SetParameter(fPID* PID, float K_p_set,float K_i_set,float K_d_set,float pLimit,float coLimit,float boost){
    (*PID).Kp = K_p_set;
    (*PID).Ki = K_i_set;
    (*PID).Kd = K_d_set;
    (*PID).pLimit = pLimit;
    (*PID).coLimit = coLimit;
    (*PID).boost = boost;
    //ips200_show_float(0, 250, K_p_set, 3, 3);
    //ips200_show_float(50, 250, (*PID).Kp, 3, 3);
    //return PID;
}
//void PID_PostionalPID(fPID* PID,int a,int AngleError){
//
//}

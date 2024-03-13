/*
 * xiao_(*PID).c
 *
 *  Created on: 2024��2��28��
 *      Author: ��
 */
#include "xiao_pid.h"
#include "xiao_steer.h"
int NORMAL_SPEED=1800;
int STOP_SPEED=0;
int CIRCLE_SPEED=0;
float setpoint_left=80.0; // setpoint_left -> left ��߱�׼�ٶ�    setpoint_right -> right  �ұ߱�׼�ٶ�
float setpoint_right=80.0;
int speedflag=1;           //ֱ����ֹͣ�������жϱ�־
extern fPID Trace_cameraLeftPID;                       //����߻�ȡ�����ߵ�PID
extern fPID Trace_cameraRightPID;                      //�ұ��߻�ȡ�����ߵ�PID
extern fPID Trace_cameraMidPID;                       //��+�һ�ȡ�����ߵ�PID
//fPID Trace_cameraLeftPID;                       //����߻�ȡ�����ߵ�PID
//fPID Trace_cameraRightPID;                      //�ұ��߻�ȡ�����ߵ�PID
//fPID Trace_cameraMidPID;                       //��+�һ�ȡ�����ߵ�PID

//iPID ipid_speed_left;
//iPID ipid_speed_right;

PID_TYPE pid_type=PID_ORIGIN;

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
        (*topid_steer).err =(int)(zhongxian-target);//�����ֵ�������

        //uart_printf(UART_0," topid_steer->err =  %d\n", topid_steer->err);


        if(-30>(*topid_steer).err)//��ƫ����
        {speedflag =2;
            //(*topid_steer).Kp=6;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp* (*topid_steer).err;
            if(speedflag == 1)
            {
                setpoint_left = NORMAL_SPEED+2.5*(*topid_steer).err;
                setpoint_right = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
                setpoint_left = CIRCLE_SPEED+6*(*topid_steer).err;
                setpoint_right = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint_right = STOP_SPEED;
                setpoint_left = STOP_SPEED+1*(*topid_steer).err;
            }
        }

        else if(-20>=(*topid_steer).err && (*topid_steer).err>=-30)//��ƫ�ϴ�
        {speedflag =2;
            //(*topid_steer).Kp=5;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
            if(speedflag == 1)
            {
              setpoint_left = NORMAL_SPEED+1*(*topid_steer).err;
              setpoint_right = NORMAL_SPEED;
            }
            else if(speedflag == 2)
            {
                setpoint_left = CIRCLE_SPEED+6*(*topid_steer).err;
                setpoint_right = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint_right = STOP_SPEED;
                setpoint_left = STOP_SPEED+1*(*topid_steer).err;
            }
        }
        else if(-13>(*topid_steer).err && (*topid_steer).err>-20)//��ƫ��С
        {speedflag =2;
            //(*topid_steer).Kp=5;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
            if(speedflag == 1)
            {
              setpoint_left = NORMAL_SPEED+1*(*topid_steer).err;
              setpoint_right =NORMAL_SPEED ;
            }
            else if(speedflag == 2)
            {
                setpoint_left = CIRCLE_SPEED+6*(*topid_steer).err;
                setpoint_right = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint_right = STOP_SPEED;
                setpoint_left = STOP_SPEED+1*(*topid_steer).err;
            }
        }
        else if(-13<=(*topid_steer).err && (*topid_steer).err<0)//��������ƫ
        {speedflag =1;
            //(*topid_steer).Kp=1;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
           if(speedflag == 1)
            {
              setpoint_left =NORMAL_SPEED;
              setpoint_right =NORMAL_SPEED ;
            }
              else if(speedflag == 2)
            {
              setpoint_left = setpoint_right = CIRCLE_SPEED;
            }

        }
        else if(0<=(*topid_steer).err && (*topid_steer).err<=13)//��������ƫ
        {speedflag =1;
            //(*topid_steer).Kp=1;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
           if(speedflag == 1)
            {
              setpoint_left =NORMAL_SPEED;
              setpoint_right =NORMAL_SPEED ;
            }
           else if(speedflag == 2)
            {
              setpoint_left = setpoint_right = CIRCLE_SPEED;
            }

        }
        else if(13<(*topid_steer).err && (*topid_steer).err<20)//��ƫ��С
        {speedflag =2;
            //(*topid_steer).Kp=5;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
           if(speedflag == 1)
            {
              setpoint_right =NORMAL_SPEED -1*(*topid_steer).err;
                setpoint_left =NORMAL_SPEED ;
            }
           else if(speedflag == 2)
            {
              setpoint_right = CIRCLE_SPEED-6*(*topid_steer).err;
                setpoint_left = CIRCLE_SPEED;
            }
           else if(speedflag == 5)
            {
              setpoint_right = STOP_SPEED-1*(*topid_steer).err;
                setpoint_left = STOP_SPEED;
            }
        }
        else if(20<=(*topid_steer).err && (*topid_steer).err<30)//��ƫ�ϴ�
        {speedflag =2;
            //(*topid_steer).Kp=5;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
            if(speedflag == 1)
            {
              setpoint_right =NORMAL_SPEED -1*(*topid_steer).err;
              setpoint_left =NORMAL_SPEED ;
            }
            else if(speedflag == 2)
            {
              setpoint_right = CIRCLE_SPEED-6*(*topid_steer).err;
                setpoint_left = CIRCLE_SPEED;
            }
            else if(speedflag == 5)
            {
              setpoint_right = STOP_SPEED-1*(*topid_steer).err;
                setpoint_left = STOP_SPEED;
            }
        }else if(30<=(*topid_steer).err)//��ƫ����
        {speedflag =2;
            //(*topid_steer).Kp=6;
            (*topid_steer).Kp_output_val=(*topid_steer).Kp*(*topid_steer).err;
            if(speedflag == 1)
            {
              setpoint_right =NORMAL_SPEED -2.5*(*topid_steer).err;
              setpoint_left =NORMAL_SPEED ;
            }
            else if(speedflag == 2)
            {
              setpoint_right = CIRCLE_SPEED-6*(*topid_steer).err;
              setpoint_left = CIRCLE_SPEED;
            }else if(speedflag == 5)
            {
                setpoint_right = STOP_SPEED-1*(*topid_steer).err;
                setpoint_left = STOP_SPEED;
            }
        }


        if((*topid_steer).Kp_output_val<-(*topid_steer).pLimit)(*topid_steer).Kp_output_val=-(*topid_steer).pLimit;//��Kp���õ�����ֵ�����޷�
        if((*topid_steer).Kp_output_val>(*topid_steer).pLimit)(*topid_steer).Kp_output_val=(*topid_steer).pLimit;  //��Kp���õ�����ֵ�����޷�

        //uart_printf(UART_0,"output = %f\n",dir_P_value);

        //(*topid_steer).Kd=(*topid_steer).Kd_Set;                                              //��ֵKd
        (*topid_steer).Kd_output_val=(*topid_steer).Kd*((*topid_steer).err-(*topid_steer).err_last); //�������Kd������ֵ

        (*topid_steer).output_val=(*topid_steer).boost*((*topid_steer).Kp_output_val+(*topid_steer).Kd_output_val);   //Kd����ֵ+Kp����ֵ  �������յ�����ֵ

        (*topid_steer).err_last=(*topid_steer).err;                               //���˴μ����ƫ��浽last_err��Ա�΢�ֵ�����



              //  toipid_speed_left.target_val=(int)setpoint_left;                                                                                                                           //�����ٶȻ�
              //  toipid_speed_right.target_val= (int)setpoint_right;
        //ips200_show_float(100, 250, (*topid_steer).output_val, 3, 3);
        PWMSetSteer(90-(*topid_steer).output_val);                        //������
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

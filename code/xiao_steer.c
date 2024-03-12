/*
 * xiao_steer.c
 *
 *  Created on: 2024年2月28日
 *      Author: 康
 */
#include "xiao_steer.h"
#include "cpu0_main.h"
extern fPID Trace_cameraLeftPID;                       //左边线获取的中线的PID
extern fPID Trace_cameraRightPID;                      //右边线获取的中线的PID
extern fPID Trace_cameraMidPID;                       //左+右获取的中线的PID

void steer_init(){

    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, STEER_MIDDLE);
}
void PWMSetSteer(float angle_pwm){
    float actual_duty=(angle_pwm/90.0+0.5)*500;
    if(actual_duty < STEER_LEFT)
        actual_duty = STEER_LEFT;
        if(actual_duty > STEER_RIGHT)
            actual_duty = STEER_RIGHT;
        //ips200_show_float(150, 250, actual_duty, 3, 3);
    pwm_set_duty(SERVO_MOTOR_PWM, actual_duty);
}

void Steer_PID_Left_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost){
    //PID_SetParameter(&Trace_cameraLeftPID, K_p_set, K_i_set, K_d_set, pLimit, coLimit, boost);
    Trace_cameraLeftPID.Kp = K_p_set;
    Trace_cameraLeftPID.Ki = K_i_set;
    Trace_cameraLeftPID.Kd = K_d_set;
    Trace_cameraLeftPID.pLimit = pLimit;
    Trace_cameraLeftPID.coLimit = coLimit;
    Trace_cameraLeftPID.boost = boost;
}
void Steer_PID_Right_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost){
    //PID_SetParameter(&Trace_cameraRightPID, K_p_set, K_i_set, K_d_set, pLimit, coLimit, boost);
    Trace_cameraRightPID.Kp = K_p_set;
    Trace_cameraRightPID.Ki = K_i_set;
    Trace_cameraRightPID.Kd = K_d_set;
    Trace_cameraRightPID.pLimit = pLimit;
    Trace_cameraRightPID.coLimit = coLimit;
    Trace_cameraRightPID.boost = boost;
}
void Steer_PID_Mid_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost){
    //PID_SetParameter(&Trace_cameraMidPID, K_p_set, K_i_set, K_d_set, pLimit, coLimit, boost);
    Trace_cameraMidPID.Kp = K_p_set;
    Trace_cameraMidPID.Ki = K_i_set;
    Trace_cameraMidPID.Kd = K_d_set;
    Trace_cameraMidPID.pLimit = pLimit;
    Trace_cameraMidPID.coLimit = coLimit;
    Trace_cameraMidPID.boost = boost;
    //ips200_show_float(0, 250, Trace_cameraMidPID.Kp, 3, 3);
}

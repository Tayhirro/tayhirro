/*
 * xiao_steer.c
 *
 *  Created on: 2024Äê2ÔÂ28ÈÕ
 *      Author: ¿µ
 */
#include "xiao_steer.h"
#include "cpu0_main.h"


void steer_init(){

    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, STEER_MIDDLE);
}
void PWMSetSteer(float angle_pwm){
    float actual_duty=(angle_pwm/90.0+0.5)*500;
    if(actual_duty < STEER_RIGHT)
        actual_duty = STEER_RIGHT;
        if(actual_duty > STEER_LEFT)
            actual_duty = STEER_LEFT;
    pwm_set_duty(SERVO_MOTOR_PWM, actual_duty);
}

void Steer_PID_Left_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost){
    PID_SetParameter(Trace_cameraLeftPID, K_p_set, K_i_set, K_d_set, pLimit, coLimit, boost);
}
void Steer_PID_Right_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost){
    PID_SetParameter(Trace_cameraRightPID, K_p_set, K_i_set, K_d_set, pLimit, coLimit, boost);
}
void Steer_PID_Mid_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost){
    PID_SetParameter(Trace_cameraMidPID, K_p_set, K_i_set, K_d_set, pLimit, coLimit, boost);
}

/*
 * xiao_steer.c
 *
 *  Created on: 2024Äê2ÔÂ28ÈÕ
 *      Author: ¿µ
 */
#include "xiao_steer.h"
#include "cpu0_main.h"


void steer_init(){

    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 750);
}
void PWMSetSteer(uint16 angle_pwm){
    if(angle_pwm < STEER_RIGHT)
             angle_pwm = STEER_RIGHT;
        if(angle_pwm > STEER_LEFT)
             angle_pwm = STEER_LEFT;
    pwm_set_duty(SERVO_MOTOR_PWM, angle_pwm);
}



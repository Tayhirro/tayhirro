/*
 * xiao_steer.h
 *
 *  Created on: 2024年2月28日
 *      Author: 康
 */

#ifndef CODE_XIAO_STEER_H_
#define CODE_XIAO_STEER_H_

#include "xiao_common.h"
#include "xiao_pid.h"
#include "cpu0_main.h"

#define SERVO_MOTOR_PWM             (ATOM1_CH1_P33_9)                           // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ            (50 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX           (50 )                                       // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX           (150)                                       // 定义主板上舵机活动范围 角度

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#define STEER_LEFT      2050     //舵机摆角左极限  这里的值是用于测试的，大家根据自己的舵机更改
#define STEER_MIDDLE    1880   //舵机摆角中值
#define STEER_RIGHT     1680   //舵机摆角右极限

extern void steer_init();
extern void PWMSetSteer(uint16 angle_pwm);

#endif /* CODE_XIAO_STEER_H_ */

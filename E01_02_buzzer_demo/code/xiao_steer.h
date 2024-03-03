/*
 * xiao_steer.h
 *
 *  Created on: 2024��2��28��
 *      Author: ��
 */

#ifndef CODE_XIAO_STEER_H_
#define CODE_XIAO_STEER_H_

#include "xiao_common.h"
#include "xiao_pid.h"
#include "cpu0_main.h"

#define SERVO_MOTOR_PWM             (ATOM1_CH1_P33_9)                           // ���������϶����Ӧ����
#define SERVO_MOTOR_FREQ            (50 )                                       // ���������϶��Ƶ��  �����ע�ⷶΧ 50-300

#define SERVO_MOTOR_L_MAX           (50 )                                       // ���������϶�����Χ �Ƕ�
#define SERVO_MOTOR_R_MAX           (150)                                       // ���������϶�����Χ �Ƕ�

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#define STEER_LEFT      2050     //����ڽ�����  �����ֵ�����ڲ��Եģ���Ҹ����Լ��Ķ������
#define STEER_MIDDLE    1880   //����ڽ���ֵ
#define STEER_RIGHT     1680   //����ڽ��Ҽ���

extern void steer_init();
extern void PWMSetSteer(uint16 angle_pwm);

#endif /* CODE_XIAO_STEER_H_ */

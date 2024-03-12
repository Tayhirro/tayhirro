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

#define STEER_LEFT      680    //����ڽ�����  �����ֵ�����ڲ��Եģ���Ҹ����Լ��Ķ������
#define STEER_MIDDLE    750   //����ڽ���ֵ
#define STEER_RIGHT     860    //����ڽ��Ҽ���

extern void steer_init();
extern void PWMSetSteer(float angle_pwm);
extern void Steer_PID_Left_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost);
extern void Steer_PID_Right_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost);
extern void Steer_PID_Mid_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost);
#endif /* CODE_XIAO_STEER_H_ */

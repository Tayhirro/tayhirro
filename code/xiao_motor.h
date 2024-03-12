/*
 * xiao_motor.h
 *
 *  Created on: 2023年4月29日
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_MOTOR_H_
#define CODE_XIAO_MOTOR_H_

#include "xiao_common.h"
#include "xiao_pid.h"
#include "cpu0_main.h"

#if DIRVER_WITH_DIRECTION       //带方向的驱动板
#define MOTOR_1_DIR             (ATOM0_CH6_P02_6)
#define MOTOR_1_PWM             (ATOM0_CH7_P02_7)
#define MOTOR_2_DIR             (ATOM0_CH4_P02_4)
#define MOTOR_2_PWM             (ATOM0_CH5_P02_5)
#define MOTOR_ADVANVE           (0)
#define MOTOR_RETREAT           (1)
#else                           //不带方向的驱动板
#define MOTOR_2_FORWARD         (ATOM0_CH2_P21_4)
#define MOTOR_2_BACKWARD        (ATOM0_CH3_P21_5)
#define MOTOR_1_FORWARD         (ATOM0_CH0_P21_2)
#define MOTOR_1_BACKWARD        (ATOM0_CH1_P21_3)
#endif

#define MOTOR_FREQ              (17000)


//电机的枚举
typedef enum {
    MOTOR_2 = 0x00,
    MOTOR_1 = 0x01
}MOTOR_PWM_enum;


//对外接口
void Motor_Init(void);
void Motor_SetSpeed(MOTOR_PWM_enum motor, int16 speed);
void Motor_PID_Init(void);
void Motor_pidClear(void);
void Motor_1PID_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost);
void Motor_2PID_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost);
void Motor_1SetPIDP(float setP);
void Motor_1SetPIDI(float setI);
void Motor_1SetPIDD(float setD);
//设置积分限制
void Motor_1SetPIDLimit(float pLimit);
//设置修正限幅
void Motor_1SetPIDCoLimit(float coLimt);
void Motor_2SetPIDP(float setP);
void Motor_2SetPIDI(float setI);
void Motor_2SetPIDD(float setD);
//设置积分限制
void Motor_2SetPIDLimit(float pLimit);
//设置修正限幅
//void Motor_ShowPIDPar(SCREEN_DEVICE screen);
void Motor_2SetPIDCoLimit(float coLimt);
float Motor_1PID_control(float target,float cur);
float Motor_2PID_control(float target,float cur);
//float Motor_FuzzyP(float kPSet, float target, float cur);
#endif /* CODE_XIAO_MOTOR_H_ */

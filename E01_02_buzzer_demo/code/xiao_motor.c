/*
 * xiao_motor.c
 *
 *  Created on: 2023年4月29日
 *      Author: Jayden_NaN
 */

#include "xiao_motor.h"
#include "cpu0_main.h"
#include "xiao_pid.h"
//电机PID
fPID Motor_1PID;
fPID Motor_2PID;
//逼近控制
float Motor_1preTarget;
float Motor_2preTarget;
uint8 Motor_1Pstatus = 0;
uint8 Motor_2Pstatus = 0;

/**
 * @brief   电机初始化
 * @return  NULL
 */
void Motor_Init(void) {
#if DIRVER_WITH_DIRECTION
    pwm_init(MOTOR_1_DIR, MOTOR_FREQ, 0);
    pwm_init(MOTOR_1_PWM, MOTOR_FREQ, 0);
    pwm_init(MOTOR_2_DIR, MOTOR_FREQ, 0);
    pwm_init(MOTOR_2_PWM, MOTOR_FREQ, 0);
#else
    pwm_init(MOTOR_2_FORWARD , MOTOR_FREQ, 0);
    pwm_init(MOTOR_2_BACKWARD, MOTOR_FREQ, 0);
    pwm_init(MOTOR_1_FORWARD, MOTOR_FREQ, 0);
    pwm_init(MOTOR_1_BACKWARD, MOTOR_FREQ, 0);
#endif
}

/**
 * @brief           电机设置速度
 * @parameter motor 选择的电机
 * @parameter speed 设置的速度
 * @return          NULL
 */
void Motor_SetSpeed(MOTOR_PWM_enum motor, int16 speed) {
#if DIRVER_WITH_DIRECTION   //带方向的驱动板
    if (motor == MOTOR_1) {
        if (speed < 0) {
            speed = -speed;
            pwm_set_duty(MOTOR_1_DIR, PWM_DUTY_MAX);
            pwm_set_duty(MOTOR_1_PWM, speed);
        }
        else {
            pwm_set_duty(MOTOR_1_DIR, 0);
            pwm_set_duty(MOTOR_1_PWM, speed);
        }
    }
    else if (motor == MOTOR_2) {
        if (speed < 0) {
            speed = -speed;
            pwm_set_duty(MOTOR_2_DIR, 0);
            pwm_set_duty(MOTOR_2_PWM, speed);
        }
        else {
            pwm_set_duty(MOTOR_2_DIR, PWM_DUTY_MAX);
            pwm_set_duty(MOTOR_2_PWM, speed);
        }

    }
#else                       //不带方向的驱动板
    if (motor == MOTOR_2) {
        if (speed < 0) {
            speed = -speed;
            pwm_set_duty(MOTOR_2_FORWARD, 0);
            pwm_set_duty(MOTOR_2_BACKWARD, speed);
        }
        else {
            pwm_set_duty(MOTOR_2_FORWARD, speed);
            pwm_set_duty(MOTOR_2_BACKWARD, 0);
        }
    }
    else if (motor == MOTOR_1) {
        if (speed < 0) {
            speed = -speed;
            pwm_set_duty(MOTOR_1_FORWARD, 0);
            pwm_set_duty(MOTOR_1_BACKWARD, speed);
        }
        else {
            pwm_set_duty(MOTOR_1_FORWARD, speed);
            pwm_set_duty(MOTOR_1_BACKWARD, 0);
        }
    }
#endif
}
/**
 * @brief           电机PID初始化
 * @return          NULL
 */
void Motor_PID_Init(void){
    PID_Init(Motor_1PID);
    PID_Init(Motor_2PID);
}
/**
 * @brief           电机PID参数设置
 * @return          NULL
 */
void Motor_1PID_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost){
    PID_SetParameter(&Motor_1PID, K_p_set, K_i_set, K_d_set, pLimit, coLimit, boost);
}
void Motor_2PID_Set(float K_p_set, float K_i_set, float K_d_set,float pLimit, float coLimit, float boost){
    PID_SetParameter(&Motor_2PID, K_p_set, K_i_set, K_d_set, pLimit, coLimit, boost);
}

void Motor_1SetPIDP(float setP) {
    Motor_1PID.Kp = setP;
    Motor_1PID.Kp_Set = setP;
}

void Motor_1SetPIDI(float setI) {
    Motor_1PID.Ki = setI;
    Motor_1PID.Ki_Set = setI;
}


void Motor_1SetPIDD(float setD) {

    Motor_1PID.Kd = setD;
    Motor_1PID.Kd_Set = setD;
}

//设置积分限制
void Motor_1SetPIDLimit(float pLimit) {
    Motor_1PID.sumLimit = pLimit;
}

//设置修正限幅
void Motor_1SetPIDCoLimit(float coLimt) {
    Motor_1PID.utLimit = coLimt;
}

void Motor_2SetPIDP(float setP) {

    Motor_2PID.Kp = setP;
    Motor_2PID.Kp_Set = setP;
}

void Motor_2SetPIDI(float setI) {
    Motor_2PID.Ki = setI;
    Motor_2PID.Ki_Set = setI;
}

void Motor_2SetPIDD(float setD) {
    Motor_2PID.Kd = setD;
    Motor_2PID.Kd_Set = setD;
}
void Motor_pidClear(){
    Motor_1PID.ut=0.0;
    Motor_2PID.ut=0.0;
}
//设置积分限制
void Motor_2SetPIDLimit(float pLimit) {
    Motor_2PID.sumLimit = pLimit;
}

//设置修正限幅
void Motor_2SetPIDCoLimit(float coLimt) {
    Motor_2PID.utLimit = coLimt;
}

/**
 * @brief               电机普通PID工作
 * @parameter LeftL     目标值
 * @parameter RightL    当前编码器值
 */
float Motor_1PID_control(float target,float cur){
    if(Motor_1Pstatus) Motor_1PID.Kp = 0;
    else Motor_1PID.Kp = Motor_1PID.Kp_Set;
    if(abs((int)target - (int)Motor_1preTarget) > 30){
        Motor_1Pstatus = 1;
    }
    if(target - cur < 7){
        Motor_1Pstatus = 0;
    }
    Motor_1PID.err=target-cur;
    Motor_1preTarget = target;
    Motor_1Puse = Motor_1PID.Kp;
    Motor_1PID.Ki = Motor_1PID.Ki_Set;
    Motor_1PID.Kd = Motor_1PID.Kd_Set;
    Motor_1PID.Kp_output_val = Motor_1PID.Kp*(Motor_1PID.err-Motor_1PID.err_last);
    Motor_1PID.Ki_output_val=Motor_1PID.Ki*Motor_1PID.err;
    Motor_1PID.Kd_output_val=Motor_1PID.Kd*(Motor_1PID.err - 2 * Motor_1PID.err_last + Motor_1PID.err_llast);
    Motor_1PID.ut=Motor_1PID.Kp_output_val+Motor_1PID.Ki_output_val+Motor_1PID.Kd_output_val;
    Motor_1PID.output_val=Motor_1PID.ut+target;
    Motor_1PID.err_llast=Motor_1PID.err_last;
    Motor_1PID.err_last=Motor_1PID.err;
  //  ////PID_SetParameter(&Motor_1PID, target, cur);
    Motor_SetSpeed(MOTOR_1, (int)Motor_1PID.output_val);
    return Motor_1PID.output_val;
}
float Motor_2PID_control(float target,float cur){
    if(Motor_2Pstatus) Motor_2PID.Kp = 0;
    else Motor_2PID.Kp = Motor_2PID.Kp_Set;
    if(abs((int)target - (int)Motor_2preTarget) > 30){
        Motor_2Pstatus = 1;
    }
    if(target - cur < 7){
        Motor_2Pstatus = 0;
    }
    Motor_2PID.err=target-cur;
    Motor_2preTarget = target;
    Motor_2Puse = Motor_2PID.Kp;
    Motor_2PID.Ki = Motor_1PID.Ki_Set;
    Motor_2PID.Kd = Motor_1PID.Kd_Set;
    Motor_2PID.Kp_output_val=Motor_2PID.Kp*(Motor_2PID.err-Motor_2PID.err_last);
    Motor_2PID.Ki_output_val=Motor_2PID.Ki*Motor_2PID.err;
    Motor_2PID.Kd_output_val=Motor_2PID.Kd*(Motor_2PID.err - 2 * Motor_2PID.err_last + Motor_2PID.err_llast);
    Motor_2PID.ut=Motor_2PID.Kp_output_val+Motor_2PID.Ki_output_val+Motor_2PID.Kd_output_val;
    Motor_2PID.output_val=Motor_2PID.ut+target;
  //  //PID_SetParameter(&Motor_2PID, target, cur);
    Motor_2PID.err_llast=Motor_2PID.err_last;
    Motor_2PID.err_last=Motor_2PID.err;
    Motor_SetSpeed(MOTOR_2, (int)Motor_2PID.output_val);
    return Motor_2PID.output_val;
}
//float Motor_FuzzyP(float kPSet, float target, float cur) {
//    Motor_ERROR_MAX = 2.5*Speed_set*Speed_diff;
//    Motor_ERROR_MIN = 0.1*Speed_set*Speed_diff;
//    if(Speed_diff < 0.1) return kPSet;
//    float error = target - cur;
//    if (error < 0) error = -error;
//    if (Motor_ERROR_MAX < 0) Motor_ERROR_MAX = -Motor_ERROR_MIN;
//    if (Motor_ERROR_MIN < 0) Motor_ERROR_MIN = -Motor_ERROR_MIN;
//    Motor_ERROR_MIN = Motor_ERROR_MIN < 5 ? Motor_ERROR_MIN : 5;
//    if (error < Motor_ERROR_MIN) return kPSet;
//    else if (error > Motor_ERROR_MAX)  return 0.0;
//    else return kPSet * (Motor_ERROR_MAX - error) / (Motor_ERROR_MAX - Motor_ERROR_MIN);
//}

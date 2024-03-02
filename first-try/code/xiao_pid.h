/*
 * xiao_pid.h
 *
 *  Created on: 2024年2月28日
 *      Author: Tayhirro
 */

#ifndef CODE_XIAO_PID_H_
#define CODE_XIAO_PID_H_
struct pidcc_controller_float{
    float Kp;
    float Ki;
    float Kd;

    float Kp_Set;
    float Ki_Set;
    float Kd_Set;
    float target_val;
    float Kp_output_val;
    float Ki_output_val;
    float Kd_output_val;
  float output_val;
    float input_val;

    float err;
    float err_last;
    float err_llast;
    float integral;
    float ut;      //
    float omin;
    float omax;
    int sumLimit;
    int utLimit;
    float pLimit;
    float coLimit;
    float boost;
};
struct pidcc_controller_int{
    float Kp;
    float Ki;
    float Kd;

    int target_val;
    int Kp_output_val;
    int Ki_output_val;
    int Kd_output_val;
  int output_val;
    int input_val;

    int err;
    int err_last;
    int integral;

    int omin;
    int omax;
};
typedef struct pidcc_controller_float fPID,*tofPID;
typedef struct pidcc_controller_int iPID,*toiPID;
extern iPID ipid_speed_left;
extern iPID ipid_speed_right;
extern fPID* pid_steer;
extern void direction_control(fPID* topid_steer,iPID* toipid_speed_left,iPID* toipid_speed_right,float zhongxian,float target);
extern void Steer_PID_Init(void);
extern float setpoint_left; // setpoint_left -> left 左边标准速度    setpoint_right -> right  右边标准速度
extern float setpoint_right;

void PID_Init(fPID PID);
void PID_SetParameter(fPID* PID, float K_p_set,float K_i_set,float K_d_set,float pLimit,float coLimit,float boost);

#endif /* CODE_XIAO_PID_H_ */

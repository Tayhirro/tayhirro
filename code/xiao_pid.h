/*
 * xiao_pid.h
 *
 *  Created on: 2024��2��28��
 *      Author: Tayhirro
 */

#ifndef CODE_XIAO_PID_H_
#define CODE_XIAO_PID_H_
//typedef struct pid{
//            float kP;
//            float kPSet;
//            float kI;
//            float kISet;
//            float kD;
//            float kDSet;
//            int sumLimit;
//            int utLimit;
//            int ut;
//}PID;

typedef enum {
        PID_ORIGIN=0x01,
        PID_INV=0x02,
}PID_TYPE;

extern PID_TYPE pid_type;

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
    int sumLimit;
    int utLimit;
    float pLimit;
    float coLimit;
    float boost;
};

typedef struct pidcc_controller_float fPID,*tofPID;
//typedef struct pidcc_controller_int iPID,*toiPID;
//extern iPID ipid_speed_left;
//extern iPID ipid_speed_right;


extern fPID Trace_cameraLeftPID;                       //����߻�ȡ�����ߵ�PID
extern fPID Trace_cameraRightPID;    /*
 * xiao_pid.h
 *
 *  Created on: 2024��2��28��
 *      Author: Tayhirro
 */

#ifndef CODE_XIAO_PID_H_
#define CODE_XIAO_PID_H_
//typedef struct pid{
//            float kP;
//            float kPSet;
//            float kI;
//            float kISet;
//            float kD;
//            float kDSet;
//            int sumLimit;
//            int utLimit;
//            int ut;
//}PID;

typedef enum {
        PID_ORIGIN=0x01,
        PID_INV=0x02,
}PID_TYPE;

extern PID_TYPE pid_type;

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
    int sumLimit;
    int utLimit;
    float pLimit;
    float coLimit;
    float boost;
};

typedef struct pidcc_controller_float fPID,*tofPID;
//typedef struct pidcc_controller_int iPID,*toiPID;
//extern iPID ipid_speed_left;
//extern iPID ipid_speed_right;


extern fPID Trace_cameraLeftPID;                       //����߻�ȡ�����ߵ�PID
extern fPID Trace_cameraRightPID;                      //�ұ��߻�ȡ�����ߵ�PID
extern fPID Trace_cameraMidPID;                       //��+�һ�ȡ�����ߵ�PID



extern void direction_control(fPID* topid_steer,float zhongxian,float target);
extern void Steer_PID_Init(void);
extern float setpoint_left; // setpoint_left -> left ��߱�׼�ٶ�    setpoint_right -> right  �ұ߱�׼�ٶ�
extern float setpoint_right;

void PID_Init(fPID* PID);
void PID_SetParameter(fPID* PID, float K_p_set,float K_i_set,float K_d_set,float pLimit,float coLimit,float boost);

#endif /* CODE_XIAO_PID_H_ */
                  //�ұ��߻�ȡ�����ߵ�PID
extern fPID Trace_cameraMidPID;                       //��+�һ�ȡ�����ߵ�PID



extern void direction_control(fPID* topid_steer,float zhongxian,float target);
extern void Steer_PID_Init(void);
extern float setpoint_left; // setpoint_left -> left ��߱�׼�ٶ�    setpoint_right -> right  �ұ߱�׼�ٶ�
extern float setpoint_right;

void PID_Init(fPID* PID);
void PID_SetParameter(fPID* PID, float K_p_set,float K_i_set,float K_d_set,float pLimit,float coLimit,float boost);

#endif /* CODE_XIAO_PID_H_ */
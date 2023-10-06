#ifndef __PIDCC_H
#define __PIDCC_H 
struct pidcc_controller_float{
	float Kp;
	float Ki;
	float Kd;
	
	float target_val;
	float Kp_output_val;
	float Ki_output_val;
	float Kd_output_val;
  float output_val;
	float input_val;
	
	float err;
	float err_last;
	float integral;
	
	float omin;
	float omax;
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
typedef struct pidcc_controller_float PID,*toPID;
typedef struct pidcc_controller_int iPID,*toiPID;
extern iPID ipid_speed_left;
extern iPID ipid_speed_right;
extern toiPID toipid_speed_left;
extern toiPID toipid_speed_right;
extern PID pid_steer;
extern void PID_param_init();
extern int speedleft_pid_realize(iPID *topid_left_speed ,int actual_val_left );
extern int speedright_pid_realize(iPID *toipid_right_speed,int actual_val_right);
extern void ele_direction_control(PID* topid_steer,iPID* toipid_speed_left,iPID* toipid_speed_right,float adc_left_mid,float adc_right_mid);
extern void pid_steers();
extern void judge_circle();
extern int judge_circle_out(float adc_left_edge,float adc_right_edge,float adc_mid);
extern int judge_circle_in(float adc_left_edge,float adc_right_edge,float adc_mid);
extern void  speed_pid();










#endif 
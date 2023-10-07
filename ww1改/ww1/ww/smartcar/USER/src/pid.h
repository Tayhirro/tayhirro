#ifndef __PID_H
#define __PID_H 



extern float setpoint1;
extern float setpoint2;
extern float p1;
extern float p2;
extern void pid_compute_new(float in1,float in2);
extern void ele_direction_control();//方向环

//typedef unsigned short int uint16; /* 16 bits */
/*-------------------------------------------------------------*/
/*		Typedefs enums & structs			*/
/*-------------------------------------------------------------*/

struct pid_controller {
	// Input, output and setpoint
	float * input; //!< Current Process Value
	float * output; //!< Corrective Output from PID Controller
	float * setpoint; //!< Controller Setpoint
	// Tuning parameters
	float Kp; //!< Stores the gain for the Proportional term
	float Ki; //!< Stores the gain for the Integral term
	float Kd; //!< Stores the gain for the Derivative term
	// Output minimum and maximum values
	float omin; //!< Maximum value allowed at the output
	float omax; //!< Minimum value allowed at the output
	// Variables for PID algorithm
	float iterm; //!< Accumulator for integral term积分
	float lastin; //!< Last input value for differential term微分
};
typedef unsigned short int	uuint16; /* 16 bits */

typedef struct pid_controller * pid_t;
extern uuint16 Bmq_last,Bmq,ideal_Bmq;   //PID理想速度设置，（编码器速度）


/*-------------------------------------------------------------*/
/*		Function prototypes				*/
/*-------------------------------------------------------------*/
	/**
	 * @brief Creates a new PID controller
	 *
	 * Creates a new pid controller and initializes it?s input, output and internal
	 * variables. Also we set the tuning parameters
	 *
	 * @param pid A pointer to a pid_controller structure
	 * @param in Pointer to float value for the process input
	 * @param out Poiter to put the controller output value
	 * @param set Pointer float with the process setpoint value
	 * @param kp Proportional gain
	 * @param ki Integral gain
	 * @param kd Diferential gain
	 *
	 * @return returns a pid_t controller handle
	 */
	pid_t pid_create(pid_t pid, float* in, float* out, float* set, float kp, float ki, float kd, float omin, float omax);

	/**
	 * @brief Computes the output of the PID control
	 *
	 * This function computes the PID output based on the parameters, setpoint and
	 * current system input.
	 *
	 * @param pid The PID controller instance which will be used for computation
	 */
	void pid_compute(pid_t pid);
        
extern void pid_compute_new(float in1,float in2);

#endif 
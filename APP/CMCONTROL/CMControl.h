#ifndef _CMCONTROL_H_
#define _CMCONTROL_H_

#include "main.h"


/* balancing robot parameters */
typedef struct balancing_robot_chassis_model_s {
	
	// static scalar parameters
	float M_bottom;   	  // mass of the lower part of the chassis (kg)
	float m_top;       	  // mass of the upper part of the chassis (kg)
	float b;           	  // estimate of viscous friction coefficient (N-m-s)
	float h;          	  // length from top of the robot to line passing through the two wheels (m)
	float I;				    	// moment of inertia of the pendulum (kg*m^2), estimated with I=(1/3)*(M+m)*h^2 or I=(1/3)*M*h^2
	float g;          	  // acceleration due to gravity (m/s^2)
	float l;        	    // length between the pendulum center of mass and the line passing through the two wheels (m)
	float r;      	      // wheel radius (m)
	float k_T_F;		      // coefficient that, given a torque T, computes the linear force to the wheels F = k_T_F*T = (2/r)*T
	
	// static matrices
	float A[4][4];				/* entries of the discrete dynamics matrix (A) */
	float B[4];		 				/* entries of the discrete inputs matrix (B) */
	float C[4][4];				/* entries of the outputs matrix (C) */
	float Vd[4][4];				/* entries of the disturbance covariance matrix (Vd) */
	float Vn[4][4];				/* entries of the noise covariance matrix (Vn) */
	
	// matrices that change at every KF iteration, but we don't need to keep track of their values for next iterations
	float Kf[4][4];				/* initial entries of the Kalman filter matrix (Kf) */
	float tmp[4][4];			/* entries of 'tmp' matrix (matrix that temporarily stores values waiting to be used for other operations) */
	float state_pred[4];	/* predictions of the robot's state */
	float P_pred[4][4];		/* entries of the predicted error covariance matrix of 'x - x_estim' (P_pred) */
	float APA[4][4];			/* entries of the matrix A*P*A^(T) (at every iteration of the Kalman Filter it gets computed from scratch) */
	float CPC[4][4];			/* entries of the matrix C*P_prec*C^(T) (at every iteration of the Kalman Filter it gets computed from scratch) */
	float S[4][4];				/* entries of the innovation covariance matrix (S) */
	float S_array_format[16];
	float S_inv[4][4];		/* entries of the inversed innovation covariance matrix (S^(-1)) */
	float S_inv_array_format[16];
	int16_t torque_to_wheel;/* input to the wheels (u) */
	float y[4];							/* output measurements array (y) */
	float delta_y[4];				/* entries of the output prediction error array (delta_y) */

	// matrices that change at every KF iteration, and we have to keep track of their values
	float state_estim_L[4];	/* estimations of the robot's state for the left wheel */
	float state_estim_R[4];	/* estimations of the robot's state for the left wheel */
	float P_L[4][4];				/* entries of the error covariance matrix of 'x - x_estim' (P) for the left wheel */
	float P_R[4][4];				/* entries of the error covariance matrix of 'x - x_estim' (P) for the right wheel */
	
	// reference signals
	float ref_L[4];
	float ref_R[4];
	float ref_L_prev[4];
	float ref_R_prev[4];
	
	// control signals
	float control_signal_L;
	float control_signal_R;
	
	//SMC parameters
	float a_smc_Pitch;		//value a(x) used for SMC (Sliding Mode Control) of Pitch and Pitch_dot
	float b_smc_Pitch;		//value b(x) used for SMC (Sliding Mode Control) of Pitch and Pitch_dot
	float a_smc_Pos;			//value a(x) used for SMC (Sliding Mode Control) of Pos and Pos_dot
	float b_smc_Pos;			//value b(x) used for SMC (Sliding Mode Control) of Pos and Pos_dot
	
} balancing_robot_chassis_model_t;


extern balancing_robot_chassis_model_t BR_chassis;
extern float *state_estim;
extern float Pitch_balance_offset;


void balancing_robot_chassis_model_initialization(balancing_robot_chassis_model_t *model);
void balancing_robot_chassis_state_pred(balancing_robot_chassis_model_t *model, int wheel_num);





void CMControlInit(void);
void CMPID_Init(void);
void CMControlOut(int16_t speedA , int16_t speedB ,int16_t speedC ,int16_t speedD );
float CMSpeedLegalize(float MotorCurrent , float limit);
void move(int16_t speedX, int16_t speedY, int16_t rad);
void key_move(int16_t speedX, int16_t speedY, int16_t rad);
void CMControlLoop(void);
void CMStop(void);
int16_t followValCal(float Setposition);
void keyboardmove(uint16_t keyboardvalue,uint16_t xlimit,uint16_t ylimit);
//void SwingFollowValCal(void);
void CM_Switch_Moni(void);
void CM_Normal_PID(void);
void CM_Climb_PID(void);
float caculate_balance(float Setposition);
//void move_balance(int16_t speedY, int16_t rad);
void balancing_robot_control(int16_t cmdForwBack, int16_t cmdRightLeft);
//void LQR_controller(void);
//void sliding_mode_controller(void);
//void kalman_filter_nonlinear(int16_t u, float y1, float y2, float y3, float y4, float *y1_estim, float *y2_estim, float *y3_estim, float *y4_estim);
//void kalman_filter_nonlinear(int robot_num, int nx, int16_t *u, int nu, float *y, int ny);
//void computeContiguousEncoderPos();
//float float_abs(float x);
//int matrix_4x4_inverse(float *m, float *m_inverse);
//void system_matrices_initialization(void);

extern float speed_limite;
extern u8 quick_spin_flag;
extern u8 climb_mode_flag;
extern u8 super_cap_flag;
extern float output_current_sum;
extern float lec_numA;
extern float lec_numB;
extern float lec_numC;
extern float lec_numD;
extern int16_t speedA_final,speedB_final,speedC_final,speedD_final;
extern float input_to_wheels;		// input to the wheels of balancing robot (to be used in Kalman filter)


#endif

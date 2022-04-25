#ifndef __Gimbal_Control_H
#define __Gimbal_Control_H


/* balancing robot parameters */
typedef struct balancing_robot_gimbal_model_s {
	
	// static scalar parameters
	float I1z;						// inertia of bottom motor of the gimbal along z axis (kg*m^2)
	float I2x;						// inertia of top motor of the gimbal along x axis (kg*m^2)
	float I2y;						// inertia of top motor of the gimbal along y axis (kg*m^2)
	float I2z;						// inertia of top motor of the gimbal along z axis (kg*m^2)
	float fv1;						// friction coefficient in the joint of the bottom motor
	float fv2;						// friction coefficient in the joint of the top motor
	
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
	int16_t torque_to_motor_1;	/* input to the bottom motor (u) */
	int16_t torque_to_motor_2;	/* input to the top motor (u) */
	float y[4];							/* output measurements array (y) */
	float delta_y[4];				/* entries of the output prediction error array (delta_y) */

	// matrices that change at every KF iteration, and we have to keep track of their values
	float state_estim[4];		/* estimations of the gimbal's state */
	float P[4][4];					/* entries of the error covariance matrix of 'x - x_estim' (P) */
	
	// reference signals
	float ref[4];
	float ref_prev[4];
	
	// control signals
	float control_signal_1;
	float control_signal_2;
	
	//SMC parameters
	float a_smc_Pitch_1;		//value a(x) used for SMC (Sliding Mode Control) of Pitch and Pitch_dot of bottom motor
	float b_smc_Pitch_1;		//value b(x) used for SMC (Sliding Mode Control) of Pitch and Pitch_dot of bottom motor
	float a_smc_Pitch_2;		//value a(x) used for SMC (Sliding Mode Control) of Pitch and Pitch_dot of top motor
	float b_smc_Pitch_2;		//value b(x) used for SMC (Sliding Mode Control) of Pitch and Pitch_dot of top motor
	
} balancing_robot_gimbal_model_t;


extern balancing_robot_gimbal_model_t BR_gimbal;
extern float *state_estim;


void balancing_robot_gimbal_model_initialization(balancing_robot_gimbal_model_t *model);
void balancing_robot_gimbal_state_pred(balancing_robot_gimbal_model_t *model);














typedef struct
{
	PID_Type Position;
	PID_Type Speed;	
}GimbalPID;

typedef struct
{
	float Mechanical;
	float Gyroscope;
}TargetType;
extern float yaw_com;
extern float pit_com;
extern int16_t position_yaw_relative; 
extern int16_t position_pit_relative;
extern TargetType YawTarget,PitchTarget;

extern float PIDOut_Whole_Pit, PIDOut_Whole_Yaw;
extern float Compensation_spin;

int16_t GimbalValLigal(int raw_gimbal_data,int middle_data);
void GimbalStop(void);
float MotorCurrentLegalize(float MotorCurrent , float limit);
void TurnToNormalPID(void);
void TurnToPreparePID(void);
void TurnToSmallANGPID(void);
void TurnToYawMechPID(void);
void TurnToBigBuffPID(void);
float YawPID_Gyro(float SetPosition);
float PitchPID_MechanicalAngle(float SetPosition);

float PitchPID_AutoAimAngle(float SetPosition);

float YawPID_MechanicalAngle(float SetPosition);
float PitchPID_MechanicalAngle(float SetRelative);
float YawPID_MechanicalAngle_Relative(float SetRelative);
float PitchPID_MechanicalAngle_Relative(float SetRelative);
float PitchPID_BigBuff(float SetPosition);
float YawPID_BigBuff(float SetPosition);
void GimbalControlLoop(void);
void GimbalControlInit(void);
void target_offset(u8 flag);
void TargetCacul(void);
#endif




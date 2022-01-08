#ifndef __Gimbal_Control_H
#define __Gimbal_Control_H


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

extern int16_t position_yaw_relative; 
extern int16_t position_pit_relative;
extern TargetType YawTarget,PitchTarget;

int16_t GimbalValLigal(int raw_gimbal_data,int middle_data);
void GimbalStop(void);
float MotorCurrentLegalize(float MotorCurrent , float limit);
void TurnToNormalPID(void);
void TurnToPreparePID(void);
void TurnToSmallANGPID(void);
float YawPID_Gyro(float SetPosition);
float PitchPID_MechanicalAngle(float SetPosition);
float YawPID_MechanicalAngle(float SetPosition);
float PitchPID_MechanicalAngle(float SetRelative);
float YawPID_MechanicalAngle_Relative(float SetRelative);
float PitchPID_MechanicalAngle_Relative(float SetRelative);
void GimbalControlLoop(void);
void GimbalControlInit(void);
void target_offset(u8 flag);
void TargetCacul(void);
#endif




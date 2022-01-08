#include "main.h"
/*******PID CONSTANT********/
float Yaw_Prepar_pp=15.0;
float Yaw_Prepar_pi=0.0;
float Yaw_Prepar_pd=6.0;
float Yaw_Prepar_sp=25.0;
float Yaw_Prepar_si=0.0;
float Yaw_Prepar_sd=6.0;

float Pitch_Prepar_pp=15.0;
float Pitch_Prepar_pi=0.0;
float Pitch_Prepar_pd=2.0;
float Pitch_Prepar_sp=13.0;
float Pitch_Prepar_si=0.0;
float Pitch_Prepar_sd=1.0;

float Yaw_Normal_pp=18.0;
float Yaw_Normal_pi=7.0;
float Yaw_Normal_pd=8.0;
float Yaw_Normal_sp=20.0;
float Yaw_Normal_si=2.0;
float Yaw_Normal_sd=8.0;

//float Yaw_Normal_pp=18.0;
//float Yaw_Normal_pi=7.0;
//float Yaw_Normal_pd=8.0;
//float Yaw_Normal_sp=20.0;
//float Yaw_Normal_si=2.0;
//float Yaw_Normal_sd=8.0;

float Yaw_SmallANG_pp=15.0;
float Yaw_SmallANG_pi=0.0;
float Yaw_SmallANG_pd=0.0;
float Yaw_SmallANG_sp=15.0;
float Yaw_SmallANG_si=0.0;
float Yaw_SmallANG_sd=0.0;

//float Yaw_SmallANG_pp=35.0;
//float Yaw_SmallANG_pi=5.0;
//float Yaw_SmallANG_pd=15.0;
//float Yaw_SmallANG_sp=28.0;
//float Yaw_SmallANG_si=1.5;
//float Yaw_SmallANG_sd=18.0;

float Pitch_Normal_pp=20.0;
float Pitch_Normal_pi=5.0;
float Pitch_Normal_pd=5.0;
float Pitch_Normal_sp=30.0;
float Pitch_Normal_si=0.5;
float Pitch_Normal_sd=3.0;
/***************************/


int16_t position_yaw_relative = 0; 
int16_t position_pit_relative = 0;

GimbalPID YawPID,PitchPID;
TargetType YawTarget,PitchTarget;
u8 target_offset_flag=0;


float PIDOut_Position_Pit,PIDOut_Speed_Pit,PIDOut_Whole_Pit;
float PIDOut_Position_Yaw,PIDOut_Speed_Yaw,PIDOut_Whole_Yaw;

int switch_mode_delay=0;
int switch_mode_delay_init=500;


void GimbalControlInit(void)
{
	PID_ControllerInit(&YawPID.Position,120,120,50000,0.001);
	PID_ControllerInit(&YawPID.Speed,50,50,5000,0.001);
	PID_ControllerInit(&PitchPID.Position,120,120,50000,0.001);
	PID_ControllerInit(&PitchPID.Speed,50,50,5000,0.001);
	PID_SetGains(&YawPID.Position,Yaw_Prepar_pp,Yaw_Prepar_pi,Yaw_Prepar_pd);
	PID_SetGains(&YawPID.Speed,Yaw_Prepar_sp,Yaw_Prepar_si,Yaw_Prepar_sd);
	PID_SetGains(&PitchPID.Position,Pitch_Prepar_pp,Pitch_Prepar_pi,Pitch_Prepar_pd);
	PID_SetGains(&PitchPID.Speed,Pitch_Prepar_sp,Pitch_Prepar_si,Pitch_Prepar_sd);
}



void GimbalControlLoop(void)
{	
	position_yaw_relative = GimbalValLigal(current_position_205	,	MIDDLE_YAW);
	position_pit_relative = GimbalValLigal(current_position_206	,	MIDDLE_PITCH);
	target_offset(target_offset_flag);
	if(remoteState==PREPARE_STATE)
	{
			target_offset_flag=1;
			switch_mode_delay=-1;
			if(remoteState_pre!=PREPARE_STATE)
			{
				TurnToPreparePID();
			}
			if(TIM5_time_count>1000)
			{
				PIDOut_Whole_Yaw=YawPID_MechanicalAngle_Relative(0);
				PIDOut_Whole_Pit=PitchPID_MechanicalAngle_Relative(0);
			}
	}
	else if(remoteState==ERROR_STATE)
	{
			switch_mode_delay=-1;
			target_offset_flag=1;
			GimbalStop();
	}
	else if(remoteState==STANDBY_STATE)
	{
			switch_mode_delay=-1;
			target_offset_flag=1;
			GimbalStop();
	}
	else if(remoteState==NORMAL_REMOTE_STATE)
	{
//			PIDOut_Whole_Yaw=YawPID_MechanicalAngle_Relative(0);
//			PIDOut_Whole_Pit=PitchPID_MechanicalAngle_Relative(0);

			if((remoteState_pre!=NORMAL_REMOTE_STATE)&&(remoteState_pre!=KEY_REMOTE_STATE))
			{
				TurnToPreparePID();
				switch_mode_delay=switch_mode_delay_init;
			}
			
			if(switch_mode_delay>0)
			{
				switch_mode_delay--;
				target_offset_flag=1;
				PIDOut_Whole_Yaw=YawPID_MechanicalAngle_Relative(0);
				PIDOut_Whole_Pit=PitchPID_MechanicalAngle_Relative(0);
			}
			else if(switch_mode_delay==0)
			{
				TurnToNormalPID();
				switch_mode_delay=-1;
			}//云台切换模式软起动归中
			else
			{
				target_offset_flag=0;
				TargetCacul();
				PIDOut_Whole_Pit=PitchPID_MechanicalAngle(PitchTarget.Mechanical);
				
				
//				if(abs(YawTarget.Gyroscope-Yaw*57.3f)<1)
//				{
//				TurnToSmallANGPID();
//				}
//				else
//				{
					TurnToNormalPID();
//				}				
				PIDOut_Whole_Yaw=YawPID_Gyro(YawTarget.Gyroscope);
			}//正常遥控器模式云台控制
	}
	else if(remoteState==KEY_REMOTE_STATE)
	{
		if((remoteState_pre!=KEY_REMOTE_STATE)&&(remoteState_pre!=NORMAL_REMOTE_STATE))
			{
				TurnToPreparePID();
				switch_mode_delay=switch_mode_delay_init;
			}
			
			if(switch_mode_delay>0)
			{
				switch_mode_delay--;
				target_offset_flag=1;
				PIDOut_Whole_Yaw=YawPID_MechanicalAngle_Relative(0);
				PIDOut_Whole_Pit=PitchPID_MechanicalAngle_Relative(0);
			}
			else if(switch_mode_delay==0)
			{
				TurnToNormalPID();
				switch_mode_delay=-1;
			}//云台切换模式软起动归中
			else
			{
				target_offset_flag=0;
				TargetCacul();
				PIDOut_Whole_Pit=PitchPID_MechanicalAngle(PitchTarget.Mechanical);
				
				if(abs(YawTarget.Gyroscope-Yaw*57.3f)<1)
				{
					TurnToSmallANGPID();
				}
				else
				{
					TurnToNormalPID();
				}				
				PIDOut_Whole_Yaw=YawPID_Gyro(YawTarget.Gyroscope);
			}
		
	}
	else if(remoteState == VIEW_STATE ) 
	{
		target_offset_flag=0;
		Auto_aim(USART_RX_BUF2,46);
		PIDOut_Whole_Pit=PitchPID_MechanicalAngle(PitchTarget.Mechanical);
				
		if(abs(YawTarget.Gyroscope-Yaw*57.3f)<1)
		{
			TurnToSmallANGPID();
		}
		else
		{
			TurnToNormalPID();
		}				
		PIDOut_Whole_Yaw=YawPID_Gyro(YawTarget.Gyroscope);
	}
	
	
	CAN1_Cmd_All((int16_t)PIDOut_Whole_Yaw, (int16_t)PIDOut_Whole_Pit , (int16_t)PIDOut_Whole_Shoot);
	
}


void TargetCacul(void)
{
	if(remoteState==NORMAL_REMOTE_STATE)
	{
		float pit_temp;
		//yaw输出为目标角度值
		if(position_yaw_relative<YawMax&&position_yaw_relative>YawMin)
		{
			YawTarget.Gyroscope -= 0.0002f * (RC_Ex_Ctl.rc.ch2);
			target_offset(2);
		}
		else 
			YawTarget.Gyroscope -= 0.00001f * (RC_Ex_Ctl.rc.ch2);

			
			//pitch输出为目标码盘值
		
		pit_temp=PitchTarget.Mechanical+0.003f	*	(RC_Ex_Ctl.rc.ch3);
		if (pit_temp<MIDDLE_PITCH+PitMax&&pit_temp>MIDDLE_PITCH+PitMin)
			PitchTarget.Mechanical=pit_temp;
//			if(position_pit_relative<PitMax&&position_pit_relative>PitMin)
//			{
//				float pit_temp;
//				pit_temp=position_pit_relative+0.003f	*	(RC_Ex_Ctl.rc.ch3);
//				if(pit_temp<PitMax&&pit_temp>PitMin)
//					PitchTarget.Mechanical +=0.003f	*	(RC_Ex_Ctl.rc.ch3);
//			}
//			else if(position_pit_relative>PitMax)
//				PitchTarget.Mechanical=MIDDLE_PITCH+PitMax-15;
//			else if(position_pit_relative<PitMin)
//				PitchTarget.Mechanical=MIDDLE_PITCH+PitMin+15;
	}
	else if(remoteState==KEY_REMOTE_STATE)
	{
		float pit_temp;
			if(position_yaw_relative<YawMax&&position_yaw_relative>YawMin)
			{
				YawTarget.Gyroscope -= 0.00005f * (RC_Ex_Ctl.rc.ch2);
				YawTarget.Gyroscope	-= (abs(RC_Ex_Ctl.mouse.x)>40 ? (RC_Ex_Ctl.mouse.x>0 ? 40: -40 ) : RC_Ex_Ctl.mouse.x) * MOUSE_YAW_SPEED;
				target_offset(2);
			}
			else
			{
				YawTarget.Gyroscope -= (abs(RC_Ex_Ctl.mouse.x)>40 ? (RC_Ex_Ctl.mouse.x>0 ? 40: -40 ) : RC_Ex_Ctl.mouse.x) * MOUSE_YAW_SPEED*0.1f;
			}
			
			pit_temp=PitchTarget.Mechanical+0.001f	*	(RC_Ex_Ctl.rc.ch3)- MOUSE_PIT_SPEED * RC_Ex_Ctl.mouse.y;
			if (pit_temp<MIDDLE_PITCH+PitMax&&pit_temp>MIDDLE_PITCH+PitMin)
				PitchTarget.Mechanical=pit_temp;
//			
//			if(position_pit_relative<PitMax||position_pit_relative>PitMin)
//			{
//				PitchTarget.Mechanical 	+= 0.00005f	*	(RC_Ex_Ctl.rc.ch3);
//				PitchTarget.Mechanical	+= MOUSE_PIT_SPEED * RC_Ex_Ctl.mouse.y;	
//			}
//			else
//				PitchTarget.Mechanical-=1;
		
	}
}




//相对值角度pid，用于归中
float YawPID_MechanicalAngle_Relative(float SetRelative)
{
	float NowPosition=position_yaw_relative;
	float NowSpeed=0;
	if (adi_die_flag==0)
		NowSpeed=adis16470_real_data.gyro_z*57.3f;
	else
		NowSpeed=-mpu6500_real_data.Gyro_Z * 57.3f;
	
	PIDOut_Position_Yaw=PID_ControllerDriver(&YawPID.Position,SetRelative,NowPosition);
	PIDOut_Speed_Yaw=PID_ControllerDriver(&YawPID.Speed,PIDOut_Position_Yaw/20.0f,NowSpeed);
	
	return MotorCurrentLegalize(PIDOut_Speed_Yaw,5000);
}
float PitchPID_MechanicalAngle_Relative(float SetRelative)
{
	float NowPosition=position_pit_relative;
	float NowSpeed=0;
	if (adi_die_flag==0)
		NowSpeed=adis16470_real_data.gyro_y*57.3f;
	else
		NowSpeed=-mpu6500_real_data.Gyro_Y * 57.3f;
		
		
	PIDOut_Position_Pit=PID_ControllerDriver(&PitchPID.Position,SetRelative,NowPosition);
	PIDOut_Speed_Pit=PID_ControllerDriver(&PitchPID.Speed,-PIDOut_Position_Pit/20.0f,NowSpeed);
	
	return MotorCurrentLegalize(PIDOut_Speed_Pit,5000);
}
/************************************************************************************/
//机械角PID
float YawPID_MechanicalAngle(float SetPosition)
{
	
	float NowPosition=continuous_current_position_205;
	float NowSpeed=0;
	if (adi_die_flag==0)
		NowSpeed=adis16470_real_data.gyro_z*57.3f;
	else
		NowSpeed=-mpu6500_real_data.Gyro_Z * 57.3f;
	
	PIDOut_Position_Yaw=PID_ControllerDriver(&YawPID.Position,SetPosition,NowPosition);
	PIDOut_Speed_Yaw=PID_ControllerDriver(&YawPID.Speed,PIDOut_Position_Yaw,NowSpeed);
	
	return MotorCurrentLegalize(PIDOut_Speed_Yaw,3000);
}

float PitchPID_MechanicalAngle(float SetPosition)
{
	float NowPosition=continuous_current_position_206;
		float NowSpeed=0;
	if (adi_die_flag==0)
		NowSpeed=adis16470_real_data.gyro_y*57.3f;
	else
		NowSpeed=-mpu6500_real_data.Gyro_Y * 57.3f;
	
	PIDOut_Position_Pit=PID_ControllerDriver(&PitchPID.Position,SetPosition,NowPosition);
	PIDOut_Speed_Pit=PID_ControllerDriver(&PitchPID.Speed,-PIDOut_Position_Pit/20.0f,NowSpeed);
	
	return MotorCurrentLegalize(PIDOut_Speed_Pit,5000);
}
/***************************************************************************************/
//陀螺仪PID
float angle_in_degree=0;
float YawPID_Gyro(float SetPosition)
{
	float NowPosition=Yaw*57.3f;
	angle_in_degree=NowPosition;
	float NowSpeed=delay_speed;
	
	SetPosition = SetPosition*22.756f;//8192/360.0f;									
	NowPosition = NowPosition*22.756f;//8192/360.0f;
	
	PIDOut_Position_Yaw=PID_ControllerDriver(&YawPID.Position,SetPosition,NowPosition);
//	if(abs(SetPosition-NowPosition)<30)
//	{
		PIDOut_Speed_Yaw=PID_ControllerDriver_test(&YawPID.Speed,PIDOut_Position_Yaw/20.0f,NowSpeed);
//	}
//	else
//	{
//		PIDOut_Speed_Yaw=PID_ControllerDriver(&YawPID.Speed,PIDOut_Position_Yaw/20.0f,NowSpeed);
//	}
	return MotorCurrentLegalize(PIDOut_Speed_Yaw,5000);
}






void TurnToPreparePID(void)
{
	PID_SetGains(&YawPID.Position,Yaw_Prepar_pp,Yaw_Prepar_pi,Yaw_Prepar_pd);
	PID_SetGains(&YawPID.Speed,Yaw_Prepar_sp,Yaw_Prepar_si,Yaw_Prepar_sd);
	PID_SetGains(&PitchPID.Position,Pitch_Prepar_pp,Pitch_Prepar_pi,Pitch_Prepar_pd);
	PID_SetGains(&PitchPID.Speed,Pitch_Prepar_sp,Pitch_Prepar_si,Pitch_Prepar_sd);
}

void TurnToNormalPID(void)
{
	PID_SetGains(&YawPID.Position,Yaw_Normal_pp,Yaw_Normal_pi,Yaw_Normal_pd);
	PID_SetGains(&YawPID.Speed,Yaw_Normal_sp,Yaw_Normal_si,Yaw_Normal_sd);
	PID_SetGains(&PitchPID.Position,Pitch_Normal_pp,Pitch_Normal_pi,Pitch_Normal_pd);
	PID_SetGains(&PitchPID.Speed,Pitch_Normal_sp,Pitch_Normal_si,Pitch_Normal_sd);
}

void TurnToSmallANGPID(void)
{
	PID_SetGains(&YawPID.Position,Yaw_SmallANG_pp,Yaw_SmallANG_pi,Yaw_SmallANG_pd);
	PID_SetGains(&YawPID.Speed,Yaw_SmallANG_sp,Yaw_SmallANG_si,Yaw_SmallANG_sd);
//	PID_SetGains(&PitchPID.Position,Pitch_Normal_pp,Pitch_Normal_pi,Pitch_Normal_pd);
//	PID_SetGains(&PitchPID.Speed,Pitch_Normal_sp,Pitch_Normal_si,Pitch_Normal_sd);
}


float MotorCurrentLegalize(float MotorCurrent , float limit)
{
	return MotorCurrent<-limit?-limit:(MotorCurrent>limit?limit:MotorCurrent);
}

void GimbalStop(void)
{
	PIDOut_Whole_Yaw=0;
	PIDOut_Whole_Pit=0;
}

int16_t GimbalValLigal(int raw_gimbal_data,int middle_data)  
{
	if((raw_gimbal_data - middle_data)>4096 )
		raw_gimbal_data -= 8192;
	if((raw_gimbal_data - middle_data)<-4096 )
		raw_gimbal_data += 8192;
	
	raw_gimbal_data -= middle_data;            
	return raw_gimbal_data;
}

void target_offset(u8 flag)
{
	switch (flag)
	{
		case 1:
			{
				YawTarget.Mechanical=continuous_current_position_205;
				YawTarget.Gyroscope=Yaw*57.3f;
				PitchTarget.Mechanical=continuous_current_position_206;
			}break;
		case 2:
		{
			YawTarget.Mechanical=continuous_current_position_205;
		}break;
		case 3:
		{
			YawTarget.Gyroscope=Yaw*57.3f;
		}break;
		case 4:
		{
			PitchTarget.Mechanical=continuous_current_position_206;
		}break;

	}
		

}


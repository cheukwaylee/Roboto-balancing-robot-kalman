#include "main.h"
/*******PID CONSTANT********/
float Yaw_Prepar_pp=15.0;
float Yaw_Prepar_pi=0.0;
float Yaw_Prepar_pd=6.0;
float Yaw_Prepar_sp=20.0;
float Yaw_Prepar_si=0.0;
float Yaw_Prepar_sd=6.0;

float Pitch_Prepar_pp=30.0;
float Pitch_Prepar_pi=0.0;
float Pitch_Prepar_pd=2.0;
float Pitch_Prepar_sp=15.0;
float Pitch_Prepar_si=0.0;
float Pitch_Prepar_sd=1.0;

float Yaw_Normal_pp=130.0;
float Yaw_Normal_pi=5.0;
float Yaw_Normal_pd=4.0;
float Yaw_Normal_sp=35.0;
float Yaw_Normal_si=0.0;
float Yaw_Normal_sd=0.0;

float Yaw_SmallANG_pp=63.0;
float Yaw_SmallANG_pi=2.0;
float Yaw_SmallANG_pd=10.0;
float Yaw_SmallANG_sp=21.5;
float Yaw_SmallANG_si=1.0;
float Yaw_SmallANG_sd=5.65;

float Yaw_Mech_pp=8.5;
float Yaw_Mech_pi=5.0;
float Yaw_Mech_pd=10.0;
float Yaw_Mech_sp=22.0;
float Yaw_Mech_si=2.3;
float Yaw_Mech_sd=10.0;


//float Pitch_Normal_pp=55.0;
//float Pitch_Normal_pi=5.5;
//float Pitch_Normal_pd=2.5;
//float Pitch_Normal_sp=70.0;
//float Pitch_Normal_si=0.0;
//float Pitch_Normal_sd=0.0;
float Pitch_Normal_pp=60.0;
float Pitch_Normal_pi=5.0;
float Pitch_Normal_pd=2.3;
float Pitch_Normal_sp=40.0;
float Pitch_Normal_si=0.0;
float Pitch_Normal_sd=0.0;

#ifdef INFANTRY_2
float Yaw_BIGBUFF_pp=130.0;//130
float Yaw_BIGBUFF_pi=5.0;//58.0 70.0       5.0
float Yaw_BIGBUFF_pd=4.0;//1.05 1.05       1.7
float Yaw_BIGBUFF_sp=35.0;//30.0
float Yaw_BIGBUFF_si=0.0;//Try to remove
float Yaw_BIGBUFF_sd=0.0;//Big buff PID

float Pitch_BIGBUFF_pp=35.0;//60 40-50-2.55 45-70-2.6 35.0 32-45-2-28-0-5 35-130-2.05-35-0-13.0
float Pitch_BIGBUFF_pi=130.0;//50 120
float Pitch_BIGBUFF_pd=2.05;//2.3 higher 3.2stop 2.6 2.4
float Pitch_BIGBUFF_sp=35.0;//40
float Pitch_BIGBUFF_si=0.0;//0.0
float Pitch_BIGBUFF_sd=13.0;//0.0
#endif

#ifdef INFANTRY_1
float Yaw_BIGBUFF_pp=130.0;//130 63
float Yaw_BIGBUFF_pi=150.0;//58.0 70.0       5.0
float Yaw_BIGBUFF_pd=3.0;//1.05 1.05       1.7
float Yaw_BIGBUFF_sp=35.0;//30.0
float Yaw_BIGBUFF_si=0.0;//Try to remove
float Yaw_BIGBUFF_sd=13.0;//Big buff PID

float Pitch_BIGBUFF_pp=60.0;//60 40-50-2.55 45-70-2.6 35.0 32-45-2-28-0-5 35-130-2.05-35-0-13.0 55-150-3.2-35-0-13
float Pitch_BIGBUFF_pi=5.0;//50 120
float Pitch_BIGBUFF_pd=1.5;//2.3 higher 3.2stop 2.6 2.4
float Pitch_BIGBUFF_sp=50.0;//40
float Pitch_BIGBUFF_si=0.0;//0.0
float Pitch_BIGBUFF_sd=13.0;//0.0
#endif

#ifdef INFANTRY_3
float Yaw_BIGBUFF_pp=90.0;//130 63
float Yaw_BIGBUFF_pi=300.0;//58.0 70.0       5.0
float Yaw_BIGBUFF_pd=3.0;//1.05 1.05       1.7
float Yaw_BIGBUFF_sp=50.0;//30.0
float Yaw_BIGBUFF_si=0.0;//Try to remove
float Yaw_BIGBUFF_sd=13.0;//Big buff PID

float Pitch_BIGBUFF_pp=100.0;//60 40-50-2.55 45-70-2.6 35.0 32-45-2-28-0-5 35-130-2.05-35-0-13.0 55-150-3.2-35-0-13
float Pitch_BIGBUFF_pi=150.0;//50 120
float Pitch_BIGBUFF_pd=3.2;//2.3 higher 3.2stop 2.6 2.4
float Pitch_BIGBUFF_sp=50.0;//40
float Pitch_BIGBUFF_si=0.0;//0.0
float Pitch_BIGBUFF_sd=13.0;//0.0
#endif

float Pitch_BIGBUFF_pp_small=50.0;//60 40-50-2.55 45-70-2.6 35.0 32-45-2-28-0-5
float Pitch_BIGBUFF_pi_small=70.0;//50 120
float Pitch_BIGBUFF_pd_small=1.6;//2.3 higher 3.2stop 2.6 2.4 3
float Pitch_BIGBUFF_sp_small=40.0;//40
float Pitch_BIGBUFF_si_small=0.0;//0.0
float Pitch_BIGBUFF_sd_small=0.0;//0.0

/***************************/
int slowdown = 0;

int16_t position_yaw_relative = 0;
int16_t position_pit_relative = 0;


GimbalPID YawPID,PitchPID;
GimbalPID YawPIDBigBuff,PitchPIDBigBuff;
TargetType YawTarget,PitchTarget;
u8 target_offset_flag=0;
u8 USB_ReadBuffer_temp[64];
int sample_time=10;

float PIDOut_Position_Pit,PIDOut_Speed_Pit,PIDOut_Whole_Pit;
float PIDOut_Position_Yaw,PIDOut_Speed_Yaw,PIDOut_Whole_Yaw;

int switch_mode_delay=0;
int switch_mode_delay_init=500;
float Compensation_spin=0;

void GimbalControlInit(void)
{
		PID_ControllerInit(&YawPID.Position,120,20000,60000,0.001);//YT,120
    PID_ControllerInit(&YawPID.Speed,50,50,30000,0.001);
    PID_ControllerInit(&PitchPID.Position,500,20000,60000,0.001);
    PID_ControllerInit(&PitchPID.Speed,50,50,30000,0.001);
    PID_SetGains(&YawPID.Position,Yaw_Prepar_pp,Yaw_Prepar_pi,Yaw_Prepar_pd);
    PID_SetGains(&YawPID.Speed,Yaw_Prepar_sp,Yaw_Prepar_si,Yaw_Prepar_sd);
    PID_SetGains(&PitchPID.Position,Pitch_Prepar_pp,Pitch_Prepar_pi,Pitch_Prepar_pd);
    PID_SetGains(&PitchPID.Speed,Pitch_Prepar_sp,Pitch_Prepar_si,Pitch_Prepar_sd);

#ifdef INFANTRY_2
    PID_ControllerInit(&YawPIDBigBuff.Position,120,20000,60000,0.001);//200,1000,5000,0.001
    PID_ControllerInit(&YawPIDBigBuff.Speed,50,50,30000,0.001);
    PID_ControllerInit(&PitchPIDBigBuff.Position,500,20000,60000,0.001);//200,750,5000,0.001
    PID_ControllerInit(&PitchPIDBigBuff.Speed,50,50,30000,0.001);
		PitchPIDBigBuff.Position.intergration_separation = 22.756*4;
    PID_SetGains(&YawPIDBigBuff.Position,Yaw_BIGBUFF_pp,Yaw_BIGBUFF_pi,Yaw_BIGBUFF_pd);
    PID_SetGains(&YawPIDBigBuff.Speed,Yaw_BIGBUFF_sp,Yaw_BIGBUFF_si,Yaw_BIGBUFF_sd);
    PID_SetGains(&PitchPIDBigBuff.Position,Pitch_BIGBUFF_pp,Pitch_BIGBUFF_pi,Pitch_BIGBUFF_pd);
    PID_SetGains(&PitchPIDBigBuff.Speed,Pitch_BIGBUFF_sp,Pitch_BIGBUFF_si,Pitch_BIGBUFF_sd);
#endif

#if ((defined INFANTRY_1) || (defined INFANTRY_3))
    PID_ControllerInit(&YawPIDBigBuff.Position,20000,20000,60000,0.001);//200,1000,5000,0.001
    PID_ControllerInit(&YawPIDBigBuff.Speed,50,50,30000,0.001);
		YawPIDBigBuff.Position.intergration_separation = 22.756*4;
    PID_ControllerInit(&PitchPIDBigBuff.Position,20000,20000,60000,0.001);//200,750,5000,0.001
    PID_ControllerInit(&PitchPIDBigBuff.Speed,50,50,30000,0.001);
		PitchPIDBigBuff.Position.intergration_separation = 22.756*4;
    PID_SetGains(&YawPIDBigBuff.Position,Yaw_BIGBUFF_pp,Yaw_BIGBUFF_pi,Yaw_BIGBUFF_pd);
    PID_SetGains(&YawPIDBigBuff.Speed,Yaw_BIGBUFF_sp,Yaw_BIGBUFF_si,Yaw_BIGBUFF_sd);
    PID_SetGains(&PitchPIDBigBuff.Position,Pitch_BIGBUFF_pp,Pitch_BIGBUFF_pi,Pitch_BIGBUFF_pd);
    PID_SetGains(&PitchPIDBigBuff.Speed,Pitch_BIGBUFF_sp,Pitch_BIGBUFF_si,Pitch_BIGBUFF_sd);
#endif
    int j=0;
    for(j=0; j<64; j++)
    {
        USB_ReadBuffer_temp[j]=0;
    }


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
        if(TIM6_time_count>2000)
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
        }//???????????
        else
        {
            target_offset_flag=0;
            TargetCacul();
            PIDOut_Whole_Pit=PitchPID_MechanicalAngle(PitchTarget.Mechanical);


//				if(abs(YawTarget.Gyroscope-Yaw*57.3f)<1)
//				{
//					TurnToSmallANGPID();
//				}
//				else
//				{
            TurnToNormalPID();
//				}
            PIDOut_Whole_Yaw=YawPID_Gyro(YawTarget.Gyroscope);
        }//???????????
    }
    else if(remoteState==KEY_REMOTE_STATE)
    {

        if((remoteState_pre!=KEY_REMOTE_STATE)&&(remoteState_pre!=NORMAL_REMOTE_STATE)&&(remoteState_pre!=VIEW_STATE))
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
        }//???????????
        else
        {
            if(quick_spin_flag==0)
            {
                target_offset_flag=0;
                TargetCacul();
                PIDOut_Whole_Pit=PitchPID_MechanicalAngle(PitchTarget.Mechanical);

//					if(abs(YawTarget.Gyroscope-Yaw*57.3f)<1)
//					{
//						TurnToSmallANGPID();
//					}
//					else
//					{
                TurnToNormalPID();
//					}
                PIDOut_Whole_Yaw=YawPID_Gyro(YawTarget.Gyroscope);
            }
            else
            {
                TurnToYawMechPID();
                target_offset_flag=1;
                PIDOut_Whole_Yaw=YawPID_MechanicalAngle_Relative(0);
                PIDOut_Whole_Pit=PitchPID_MechanicalAngle_Relative(0);
            }
        }

    }
    else if(remoteState == VIEW_STATE )
    {
			if(remoteState_pre!=VIEW_STATE)
			{
				yaw_bias=0;
				pitch_bias=0;
			}
        if(auto_aim_flag == 0xFF && big_power_flag == 0x00)//&&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_C ) == KEY_PRESSED_OFFSET_C )
        //if(1)
        {
#ifdef AutoAim_USB
            Auto_aim(USB_ReadBuffer,64);
#else
            Auto_aim(fram,15);
#endif
						
						target_offset_flag=2;
            PIDOut_Whole_Pit=PitchPID_MechanicalAngle(PitchTarget.Mechanical);
            TurnToNormalPID();
            PIDOut_Whole_Yaw=YawPID_Gyro(YawTarget.Gyroscope);
        }
				else if(big_power_flag == 0xFF && auto_aim_flag == 0x00)
				{
					#ifdef AutoAim_USB
            Auto_aim(USB_ReadBuffer,64);
#else
            Auto_aim(fram,15);
#endif				
            target_offset_flag=2;
						TurnToBigBuffPID();
						
#ifdef INFANTRY_2
            PIDOut_Whole_Pit=PitchPID_BigBuff(PitchTarget.Mechanical);
            PIDOut_Whole_Yaw=YawPID_Gyro(YawTarget.Gyroscope);
#endif
						
#if ((defined INFANTRY_1) || (defined INFANTRY_3))
            PIDOut_Whole_Pit=PitchPID_BigBuff(PitchTarget.Mechanical);
            PIDOut_Whole_Yaw=YawPID_BigBuff(YawTarget.Mechanical);
#endif
						
						
            //PIDOut_Whole_Yaw=YawPID_Gyro(YawTarget.Gyroscope);
				}
    }

    CAN1_Cmd_All((int16_t)PIDOut_Whole_Yaw, (int16_t)PIDOut_Whole_Pit);
    //CAN1_Cmd_All((int16_t)0, (int16_t)0);
}

float yaw_com=0;
float pit_com=0;
void TargetCacul(void)
{
    if(remoteState==NORMAL_REMOTE_STATE)
    {

        float pit_temp;
				if(position_yaw_relative<YawMax&&position_yaw_relative>YawMin&&(YawTarget.Gyroscope-Yaw*57.3f)<60)
				{
					YawTarget.Gyroscope -= 0.0002f * (RC_Ex_Ctl.rc.ch2);
					target_offset(2);
				}
				else 
				{
					YawTarget.Gyroscope -= 0.000001f * (RC_Ex_Ctl.rc.ch2);
		//			if(YawTarget.Gyroscope-Yaw*57.3f>60)
		//				YawTarget.Gyroscope=Yaw*57.3f+60;
		//			else if(YawTarget.Gyroscope-Yaw*57.3f<-60)
		//				YawTarget.Gyroscope=Yaw*57.3f-60;
					target_offset(2);
				}
					
        pit_temp=PitchTarget.Mechanical-0.003f	*	(RC_Ex_Ctl.rc.ch3);
        if(pit_temp>(MIDDLE_PITCH + PitMax)) {
            PitchTarget.Mechanical = MIDDLE_PITCH + PitMax;
        }
        else if(pit_temp<(MIDDLE_PITCH + PitMin)) {
            PitchTarget.Mechanical = MIDDLE_PITCH + PitMin;
        }
        else {
            PitchTarget.Mechanical=pit_temp;
        }

    }
    else if(remoteState==KEY_REMOTE_STATE)
    {
        if(clearing_flag==0)
        {
            float pit_temp;
          if(adi_die_flag==0&&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_G )==KEY_PRESSED_OFFSET_G)
					{
						;
					}
					else
					{
						YawTarget.Gyroscope -= 0.00005f *(RC_Ex_Ctl.rc.ch2);
            YawTarget.Gyroscope	-= (abs(RC_Ex_Ctl.mouse.x)>40 ? (RC_Ex_Ctl.mouse.x>0 ? 40: -40 ) : RC_Ex_Ctl.mouse.x) * MOUSE_YAW_SPEED;
            Compensation_spin=(abs(RC_Ex_Ctl.mouse.x)>40 ? (RC_Ex_Ctl.mouse.x>0 ? 40: -40 ) : RC_Ex_Ctl.mouse.x) * MOUSE_YAW_SPEED*2000.0f;
            if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_Q )==KEY_PRESSED_OFFSET_Q )
            {
                YawTarget.Gyroscope+=0.18f;
                Compensation_spin-=110;
            }
            else if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_E )==KEY_PRESSED_OFFSET_E )
            {
                YawTarget.Gyroscope-=0.18f;
                Compensation_spin=110;
            }
            else Compensation_spin=0;
					}
            target_offset(2);


            pit_temp=PitchTarget.Mechanical-0.001f	*	(RC_Ex_Ctl.rc.ch3)+ MOUSE_PIT_SPEED * RC_Ex_Ctl.mouse.y;
            if(pit_temp>(MIDDLE_PITCH + PitMax)) {
                PitchTarget.Mechanical = MIDDLE_PITCH + PitMax;
            }
            else if(pit_temp<(MIDDLE_PITCH + PitMin)) {
                PitchTarget.Mechanical = MIDDLE_PITCH + PitMin;
            }
            else {
                PitchTarget.Mechanical=pit_temp;
            }

        }
    }
    else if(remoteState==VIEW_STATE)
    {
//		float pit_temp;
//			if(position_yaw_relative<YawMax&&position_yaw_relative>YawMin)
//			{
//				YawTarget.Mechanical -= 0.00005f * (RC_Ex_Ctl.rc.ch2);
        yaw_com	=-(abs(RC_Ex_Ctl.mouse.x)>40 ? (RC_Ex_Ctl.mouse.x>0 ? 40: -40 ) : RC_Ex_Ctl.mouse.x) * MOUSE_YAW_SPEED;
//				target_offset(3);
//			}
//			else
//			{
//				YawTarget.Mechanical -= (abs(RC_Ex_Ctl.mouse.x)>40 ? (RC_Ex_Ctl.mouse.x>0 ? 40: -40 ) : RC_Ex_Ctl.mouse.x) * MOUSE_YAW_SPEED*0.1f;
//			}
//
        pit_com=MOUSE_PIT_SPEED * RC_Ex_Ctl.mouse.y;
//			if (pit_temp<MIDDLE_PITCH+PitMax&&pit_temp>MIDDLE_PITCH+PitMin)
//				PitchTarget.Mechanical=pit_temp;
    }
}




//?????pid,????
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
        NowSpeed=-adis16470_real_data.gyro_y*57.3f;
    else
        NowSpeed=mpu6500_real_data.Gyro_Y * 57.3f;

    PIDOut_Position_Pit=PID_ControllerDriver(&PitchPID.Position,SetRelative,NowPosition);
    PIDOut_Speed_Pit=PID_ControllerDriver(&PitchPID.Speed,PIDOut_Position_Pit/20.0f,NowSpeed);

    return MotorCurrentLegalize(PIDOut_Speed_Pit,5000);
}
/************************************************************************************/
//???PID
float YawPID_MechanicalAngle(float SetPosition)
{

    float NowPosition=continuous_current_position_205;
    float NowSpeed=0;
    if (adi_die_flag==0)
        NowSpeed=adis16470_real_data.gyro_z*57.3f;
    else
        NowSpeed=-mpu6500_real_data.Gyro_Z * 57.3f;

    if(fabs(SetPosition-NowPosition)<50)
        NowSpeed=NowSpeed*2.0f;

    PIDOut_Position_Yaw=PID_ControllerDriver(&YawPID.Position,SetPosition,NowPosition);
    PIDOut_Speed_Yaw=PID_ControllerDriver(&YawPID.Speed,PIDOut_Position_Yaw/20.0f,NowSpeed);

    return MotorCurrentLegalize(PIDOut_Speed_Yaw,5000);
}

float PitchPID_MechanicalAngle(float SetPosition)
{
    float NowPosition=continuous_current_position_filtered_206;
    float NowSpeed=0;//current_cm_206;
    if (adi_die_flag==0)
        NowSpeed=-adis16470_real_data.gyro_y*57.3f;
    else
        NowSpeed=-mpu6500_real_data.Gyro_Y * 57.3f;


    PIDOut_Position_Pit=PID_ControllerDriver(&PitchPID.Position,SetPosition,NowPosition);
    PIDOut_Speed_Pit=PID_ControllerDriver(&PitchPID.Speed,PIDOut_Position_Pit/20.0f,NowSpeed);

    return MotorCurrentLegalize(PIDOut_Speed_Pit,30000);
}

#ifdef INFANTRY_2
float YawPID_BigBuff(float SetPosition)
{
    float NowPosition=continuous_current_position_filtered_205;//Gimbal_control.angle_yaw_current
    float NowSpeed=0;
    if (adi_die_flag==0)
        NowSpeed=adis16470_real_data.gyro_z*57.3f;
    else
        NowSpeed=-mpu6500_real_data.Gyro_Z * 57.3f;

    PIDOut_Position_Yaw=PID_ControllerDriver(&YawPIDBigBuff.Position,SetPosition,NowPosition);// see error
    PIDOut_Speed_Yaw=PID_ControllerDriver(&YawPID.Speed,PIDOut_Position_Yaw/20.0f,NowSpeed);

    return MotorCurrentLegalize(PIDOut_Speed_Yaw,30000);
}

float PitchPID_BigBuff(float SetPosition)
{
    float NowPosition=continuous_current_position_filtered_206;
    float NowSpeed=0;//current_cm_206;
    if (adi_die_flag==0)
        NowSpeed=-adis16470_real_data.gyro_y*57.3f;
    else
        NowSpeed=-mpu6500_real_data.Gyro_Y * 57.3f;
		
		/*if(fabs(fram.pitch) > 1.0)
		//if(smallbuff == 1)
		{
			PID_SetGains(&PitchPIDBigBuff.Position,Pitch_BIGBUFF_pp,Pitch_BIGBUFF_pi,Pitch_BIGBUFF_pd);
			PID_SetGains(&PitchPIDBigBuff.Speed,Pitch_BIGBUFF_sp,Pitch_BIGBUFF_si,Pitch_BIGBUFF_sd);
		}
		else
		{
			PID_SetGains(&PitchPIDBigBuff.Position,Pitch_BIGBUFF_pp_small,Pitch_BIGBUFF_pi_small,Pitch_BIGBUFF_pd_small);
			PID_SetGains(&PitchPIDBigBuff.Speed,Pitch_BIGBUFF_sp_small,Pitch_BIGBUFF_si_small,Pitch_BIGBUFF_sd_small);
		}*/
		
		PID_SetGains(&PitchPIDBigBuff.Position,Pitch_BIGBUFF_pp,Pitch_BIGBUFF_pi,Pitch_BIGBUFF_pd);
		PID_SetGains(&PitchPIDBigBuff.Speed,Pitch_BIGBUFF_sp,Pitch_BIGBUFF_si,Pitch_BIGBUFF_sd);

    PIDOut_Position_Pit=PID_ControllerDriver(&PitchPIDBigBuff.Position,SetPosition,NowPosition);
    PIDOut_Speed_Pit=PID_ControllerDriver(&PitchPIDBigBuff.Speed,PIDOut_Position_Pit/20.0f,NowSpeed);

    return MotorCurrentLegalize(PIDOut_Speed_Pit,30000);
}
#endif

#if ((defined INFANTRY_1) || (defined INFANTRY_3))
float YawPID_BigBuff(float SetPosition)
{
    float NowPosition=continuous_current_position_filtered_205;//Gimbal_control.angle_yaw_current
    float NowSpeed=0;
    if (adi_die_flag==0)
        NowSpeed=adis16470_real_data.gyro_z*57.3f;
    else
        NowSpeed=-mpu6500_real_data.Gyro_Z * 57.3f;
		
		if(fabs(SetPosition-NowPosition)<50)
        NowSpeed=NowSpeed*2.0f;

    PIDOut_Position_Yaw=PID_ControllerDriver(&YawPIDBigBuff.Position,SetPosition,NowPosition);// see error
    PIDOut_Speed_Yaw=PID_ControllerDriver(&YawPIDBigBuff.Speed,PIDOut_Position_Yaw/20.0f,NowSpeed);

    return MotorCurrentLegalize(PIDOut_Speed_Yaw,30000);
}

float PitchPID_BigBuff(float SetPosition)
{
    float NowPosition=continuous_current_position_filtered_206;
    float NowSpeed=0;//current_cm_206;
    if (adi_die_flag==0)
        NowSpeed=-adis16470_real_data.gyro_y*57.3f;
    else
        NowSpeed=-mpu6500_real_data.Gyro_Y * 57.3f;
		
		/*if(fabs(fram.pitch) > 1.0)
		//if(smallbuff == 1)
		{
			PID_SetGains(&PitchPIDBigBuff.Position,Pitch_BIGBUFF_pp,Pitch_BIGBUFF_pi,Pitch_BIGBUFF_pd);
			PID_SetGains(&PitchPIDBigBuff.Speed,Pitch_BIGBUFF_sp,Pitch_BIGBUFF_si,Pitch_BIGBUFF_sd);
		}
		else
		{
			PID_SetGains(&PitchPIDBigBuff.Position,Pitch_BIGBUFF_pp_small,Pitch_BIGBUFF_pi_small,Pitch_BIGBUFF_pd_small);
			PID_SetGains(&PitchPIDBigBuff.Speed,Pitch_BIGBUFF_sp_small,Pitch_BIGBUFF_si_small,Pitch_BIGBUFF_sd_small);
		}*/
		
		PID_SetGains(&PitchPIDBigBuff.Position,Pitch_BIGBUFF_pp,Pitch_BIGBUFF_pi,Pitch_BIGBUFF_pd);
		PID_SetGains(&PitchPIDBigBuff.Speed,Pitch_BIGBUFF_sp,Pitch_BIGBUFF_si,Pitch_BIGBUFF_sd);

    PIDOut_Position_Pit=PID_ControllerDriver(&PitchPIDBigBuff.Position,SetPosition,NowPosition);
    PIDOut_Speed_Pit=PID_ControllerDriver(&PitchPIDBigBuff.Speed,PIDOut_Position_Pit/20.0f,NowSpeed);

    return MotorCurrentLegalize(PIDOut_Speed_Pit,30000);
}
#endif

/***************************************************************************************/
//???PID
float angle_in_degree=0;
float YawPID_Gyro(float SetPosition)
{
    float NowPosition=Yaw*57.3f;
    angle_in_degree=NowPosition;
    float NowSpeed=delay_speed;

    SetPosition = SetPosition*22.756f;//8192/360.0f;
    NowPosition = NowPosition*22.756f;//8192/360.0f;
    if(fabs(SetPosition-NowPosition)<50)
    {
//		Yaw_SmallANG_pp=(1+0.5*(1-abs(SetPosition-NowPosition)/50.0f))*40;
        NowSpeed=NowSpeed*2;//(1+3*(1-abs(SetPosition-NowPosition)/100.0f));
    }
//	else
//		Yaw_SmallANG_pp=40;



    PIDOut_Position_Yaw=PID_ControllerDriver(&YawPID.Position,SetPosition,NowPosition);
    PIDOut_Speed_Yaw=PID_ControllerDriver(&YawPID.Speed,PIDOut_Position_Yaw/20.0f,NowSpeed);

    return MotorCurrentLegalize(PIDOut_Speed_Yaw,30000);
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

void TurnToBigBuffPID(void)
{
    PID_SetGains(&YawPIDBigBuff.Position,Yaw_BIGBUFF_pp,Yaw_BIGBUFF_pi,Yaw_BIGBUFF_pd);
    PID_SetGains(&YawPIDBigBuff.Speed,Yaw_BIGBUFF_sp,Yaw_BIGBUFF_si,Yaw_BIGBUFF_sd);
    PID_SetGains(&PitchPIDBigBuff.Position,Pitch_BIGBUFF_pp,Pitch_BIGBUFF_pi,Pitch_BIGBUFF_pd);
    PID_SetGains(&PitchPIDBigBuff.Speed,Pitch_BIGBUFF_sp,Pitch_BIGBUFF_si,Pitch_BIGBUFF_sd);
}

void TurnToYawMechPID(void)
{
    PID_SetGains(&YawPID.Position,Yaw_Mech_pp,Yaw_Mech_pi,Yaw_Mech_pd);
    PID_SetGains(&YawPID.Speed,Yaw_Mech_sp,Yaw_Mech_si,Yaw_Mech_sd);
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
    }
    break;//用于无云台控制状态或归中初始化，将各种模式目标值归为实时当前值
    case 2:
    {
        YawTarget.Mechanical=continuous_current_position_205;
    }
    break;//用于用陀螺仪控制yaw轴时，机械角目标更新
    case 3:
    {
        YawTarget.Gyroscope=Yaw*57.3f;
    }
    break;//用于机械角控制yaw轴时候，陀螺仪目标角更新
    case 4:
    {
        PitchTarget.Mechanical=continuous_current_position_206;
    }
    break;//用于陀螺仪控制pitch时候，机械目标角更新

    }


}


/************************************************************
 *File		:	ShootControl.c
 *Author	:  @YUYENCHIA ,jiafish@outlook.com	
 *Version	: V1.0
 *Update	: 2018.12.11
 *Description: 	
 ************************************************************/

#include "main.h"
#define PWM_SELF_ADJUST

 

//Shooter parameter
#define Shoot_DOWN 0    
float Shoot_UP_HIGH=52;//35-21.8m/s 40-22~24m/s 45-22.5~25m/s 50-23.5~26.5m/s 55-24~28m/s 52-24~27m/s 42-22.5~24.5m/s
float Shoot_UP_LOW=30;  
#define SHOOT_DELAY 750

int auto_begin = 0;

int Shoot_UP;  //?Shoot_Para_Moni?
int Shoot_UP_Pre;// be same with initial Shoot_UP

float heat_limite=240;
float PIDOut_Position_Shoot,PIDOut_Speed_Shoot,PIDOut_Whole_Shoot;					
ShootPID_Type ShootPID;
PID_Type	MotorShoot;
Shoot_Target ShootTarget;

float shoot_pp=100.0;//80.0
float shoot_pi=3.0;//1.0
float shoot_pd=0.0;//0.0
float shoot_sp=16.0;//5.0
float shoot_si=0.0;//0.0
float shoot_sd=0.0;//0.0

float test_shootmotor_gothrought = 0.0;
float last_207 = 0.0;


//Friction parameter
#define Friction_OFF 0
#define Friction_ON  1
#define friction_count_time 40

#define Friction_SPEED_p               0.06f
#define Friction_SPEED_i               0.005f
#define Friction_SPEED_d               0.01f
#define Friction_INTE_limitI           60.f
#define DELTA_TIME                     0.01f
int16_t frictionState;
int16_t friction_count ;
u8 flag_FrtctionDelay = 0;
int friction_acce_flag=0;
int acce_delay=30;
int acce_delay_init=30;
int friction_speed_temp=Shoot_DOWN;
PWMFriction_Type    Friction_CH1;
PWMFriction_Type    Friction_CH2;

uint32_t friction_die_time;
u8 friction_die_flag=0;
uint32_t time_tick_1ms_shoot = 0;

u8 friction_acce_flag_re=0;
u8 friction_dece_flag=0;

u8 reach_flag=0;

float speed_current;
	
float last_bullet_speed;

u8 friction_state_flag=0;


//--Shoot_Para_Moni--
int bullet_added_flag=0;
int bullet_add_delay=0;
int bullet_add_delay_init=100;
int bullet_clear_count=0;
u8 shoot_up_speed_flag=0;
int shoot_speed_change_flag_count=0;
/***************************************************************************************
 *Name     : Client Layer
 *Function :
 *Input    :
 *Output   :
 *Description : 
****************************************************************************************/
void ShootControlInit(void)
{
	PidShootInit();	
	Friction_Looper(Shoot_DOWN);
	ShootStop();
}

void ShootStop(void)				
{
	Friction_Looper(Shoot_DOWN);
	friction_speed_temp=Shoot_DOWN;
	PIDOut_Whole_Shoot = 0;
	frictionState = Friction_OFF;
	TargetRenew(0);
	LASER_OFF();
	friction_state_flag=0;
}

void ShootControlLoop(void)
{
	time_tick_1ms_shoot++;
	speed_current=friction_current_speed();
	
	Shoot_Para_Moni();
	if(remoteState == PREPARE_STATE)
	{
		ShootStop();
	}
	else if(remoteState == NORMAL_REMOTE_STATE)
	{
		Friction_Moni();
		Shoot_target_Cal();
		Shooter_Moni();
		Cover_Moni();
	}
	else if(remoteState == KEY_REMOTE_STATE )  
	{
		Friction_Moni();
		Shoot_target_Cal();
		Shooter_Moni();
		Cover_Moni();
	}
	else if(remoteState == VIEW_STATE)
	{
			Friction_Moni();
			Shoot_target_Cal();
			Shooter_Moni();
	}
	else if(remoteState == STANDBY_STATE ) 
	{
		ShootStop();
	}
	else if(remoteState == ERROR_STATE ) 
	{
		ShootStop();
	}
	
	if(time_tick_1ms_shoot == 1000000)time_tick_1ms_shoot = 1;
	
	
	CAN2_Cmd_SHOOT(PIDOut_Whole_Shoot);
}


/***************************************************************************************
 *Name     : Friction
 *Function :
 *Input    :
 *Output   :
 *Description : 
****************************************************************************************/
void Friction_Moni(void)
{
	FrictionJudge();
	//FrictionControl();
	
	
	  if(friction_acce_flag==1||friction_acce_flag_re==1||friction_dece_flag==1)
		{
			acce_delay--;
		}
		
		if(friction_die_flag)// friction restart process
		{
			Fricton_restart();
			if(speed_current>Shoot_UP*0.5)friction_die_flag=0;
		}		
		
		//-- friction speed adjustment starts from here--
		if((friction_acce_flag_re==1||friction_acce_flag==1)&&acce_delay<=0&&friction_speed_temp<Shoot_UP&&frictionState==Friction_ON)
		{
      if(Shoot_UP-friction_speed_temp<5)friction_speed_temp=Shoot_UP;
      else friction_speed_temp+=5;
			acce_delay=acce_delay_init;
			Friction_Looper(friction_speed_temp);
		}
		else if(friction_dece_flag==1&&acce_delay<=0&&friction_speed_temp>Shoot_UP&&frictionState==Friction_ON)
		{
      if(friction_speed_temp-Shoot_UP<5)friction_speed_temp=Shoot_UP;
			else friction_speed_temp-=5;
			acce_delay=acce_delay_init;
			Friction_Looper(friction_speed_temp);
		}
		
		else if ((friction_acce_flag_re==1||friction_acce_flag==1)&&acce_delay<=0&&friction_speed_temp>=Shoot_UP)
		{
			friction_acce_flag=0;
			friction_acce_flag_re=0;  
			
		}
		else if (friction_dece_flag==1&&acce_delay<=0&&friction_speed_temp<=Shoot_UP)
		{
		  friction_dece_flag=0;
		}
		//-- friction speed adjustment ends at here--
		
		
	  if(speed_current>Shoot_UP-5 && speed_current<Shoot_UP+5)//friction speed reload finished
		{
			reach_flag=1;
			Shoot_UP_Pre=Shoot_UP;
		}
		

		if(reach_flag==1&&speed_current<3&&friction_acce_flag_re==0&&friction_die_flag==0)//friction death monitor
		{
			friction_die_time=time_tick_1ms_shoot;
			friction_die_flag=1;
			if(frictionState==Friction_OFF)friction_die_flag=0;
		}	
		#ifdef PWM_SELF_ADJUST
		if(big_power_flag != 0xFF)
			PWM_self_adjust();
		else
			Shoot_UP_HIGH=52;
		
		#endif
 
		
		
}



uint32_t Fricton_restart(void)
{
	if(time_tick_1ms_shoot-friction_die_time<750) 
	{
		friction_speed_temp=0;
		Friction_Looper(Shoot_DOWN);
		
	}
	else
	{		
		friction_acce_flag_re=1;
		friction_die_flag=0;
	}
		return time_tick_1ms_shoot;
}





void PWM_self_adjust(void)
{

		if(ext_shoot_data.bullet_speed>12)
		{
			if(last_bullet_speed==0)last_bullet_speed=ext_shoot_data.bullet_speed;
			else if(last_bullet_speed!=ext_shoot_data.bullet_speed)
			{
				if(big_power_flag == 0xFF)
				{
					if(ext_shoot_data.bullet_speed>27.5f)
					{
    				friction_dece_flag=1;
						Shoot_UP_HIGH-=0.5f*fabs(ext_shoot_data.bullet_speed-25.5f)<=3?0.5f*fabs(ext_shoot_data.bullet_speed-25.5f):3;
					}
					else if(ext_shoot_data.bullet_speed<23.5f)
					{
    				friction_acce_flag_re=1;
						Shoot_UP_HIGH+=0.5f*fabs(ext_shoot_data.bullet_speed-25.5f)<=3?0.5f*fabs(ext_shoot_data.bullet_speed-25.5f):3;
					}	
				}
				else {
				if(shoot_up_speed_flag==0)
				{
					if(ext_shoot_data.bullet_speed>20.0f)
					{
    				friction_dece_flag=1;
						Shoot_UP_LOW-=0.5f*fabs(ext_shoot_data.bullet_speed-18.5f)<=3?0.5f*fabs(ext_shoot_data.bullet_speed-18.5f):3;
					}
					else if(ext_shoot_data.bullet_speed<17.0f)
					{
    				friction_acce_flag_re=1;
						Shoot_UP_LOW+=0.5f*fabs(ext_shoot_data.bullet_speed-18.5f)<=3?0.5f*fabs(ext_shoot_data.bullet_speed-18.5f):3;
					}		
				}
				else if(shoot_up_speed_flag==1)
				{
					if(ext_shoot_data.bullet_speed>27.5f)
					{
    				friction_dece_flag=1;
						Shoot_UP_HIGH-=0.5f*fabs(ext_shoot_data.bullet_speed-25.5f)<=3?0.5f*fabs(ext_shoot_data.bullet_speed-25.5f):3;
					}
					else if(ext_shoot_data.bullet_speed<24.0f)
					{
    				friction_acce_flag_re=1;
						Shoot_UP_HIGH+=0.5f*fabs(ext_shoot_data.bullet_speed-25.5f)<=3?0.5f*fabs(ext_shoot_data.bullet_speed-25.5f):3;
					}
				}	
			}				
			}
			
	    
			last_bullet_speed=ext_shoot_data.bullet_speed;
			
		}
	
}


void FrictionJudge(void)
{
	if(frictionState == Friction_OFF && flag_FrtctionDelay == 0 )  
	 {                                                                                      
		if(flag_friction_switch == 1)                     
      {
			  frictionState = Friction_ON;
			  flag_friction_switch=0;
				FrictionControl();
			}   
	 }
	else if(frictionState == Friction_ON)
	 {
		if( flag_friction_switch == 1)
      {
				frictionState = Friction_OFF;
				flag_friction_switch = 0;
				FrictionControl();
			}
	 }
	if(flag_FrtctionDelay > 0) flag_FrtctionDelay--;    
}

void FrictionControl(void)
{
	if(frictionState == Friction_OFF )
	{
		Friction_Looper(Shoot_DOWN);
		PIDOut_Whole_Shoot = 0;
		friction_acce_flag=0;
		friction_acce_flag_re=0;
		friction_speed_temp=Shoot_DOWN;
		LASER_OFF();
		
	}
	if(frictionState == Friction_ON)
	{
			friction_acce_flag=1;
			LASER_ON();
	}
}

void Friction_Init(void)//???
	{
    TIM5_Init();
    PID_ControllerInit(&Friction_CH1.pid, Friction_INTE_limitI, (uint16_t)-1, 160, DELTA_TIME);
    Friction_CH1.pid.Kp = Friction_SPEED_p;
    Friction_CH1.pid.Ki = Friction_SPEED_i;
    Friction_CH1.pid.Kd = Friction_SPEED_d;
    Friction_CH1.pid.intergration_separation = 20.f;
    Friction_CH1.pid.functions.output_filter = MovingAverageFilter_f32;
    
    PID_ControllerInit(&Friction_CH2.pid, Friction_INTE_limitI, (uint16_t)-1, 160, DELTA_TIME);
    Friction_CH2.pid.Kp = Friction_SPEED_p;
    Friction_CH2.pid.Ki = Friction_SPEED_i;
    Friction_CH2.pid.Kd = Friction_SPEED_d;
    Friction_CH2.pid.intergration_separation = 20.f;
    Friction_CH2.pid.functions.output_filter = MovingAverageFilter_f32;
}


uint32_t check = 0;
int16_t temp_pwm=0;
void Friction_Looper(uint32_t target) {
	 // looperUpdateFriction(&Friction_CH1);
   // looperUpdateFriction(&Friction_CH2);
		temp_pwm=800-target;
    TIM_SetCompare4(TIM5, temp_pwm);
		
}

/***************************************************************************************
 *Name     : Shooter
 *Function :
 *Input    :
 *Output   :
 *Description : 
****************************************************************************************/
int stuck_count=0;
int inver_rotate_delay=0;
int inver_rotate_delay_init=20;

void Shooter_Moni(void)
{
	if(frictionState == Friction_ON)
	{
		if(speed_current<Shoot_UP*0.45)
		{
			friction_state_flag=0;
		}
		else
		{
			friction_state_flag=1;
		}
		if(speed_current<Shoot_UP*0.45)
		{
			BurstMove(0);
		}
		else if(friction_acce_flag_re==0&&friction_acce_flag==0)
		{
		if(remoteState == KEY_REMOTE_STATE&&inver_rotate_delay<=0)
		{
			if(RC_Ctl.mouse.press_r)
			{
				BurstMove(ShootTarget.speed);
				StuckMoni_Speed();
			}
			else
			{
				ShootMove(ShootTarget.position);
				StuckMoni();
			}
		}
		else if(remoteState == NORMAL_REMOTE_STATE&&inver_rotate_delay<=0)
		{
			
			BurstMove(ShootTarget.speed);
			StuckMoni_Speed();
		}
		else if(remoteState == VIEW_STATE&&inver_rotate_delay<=0)
		{
			if(big_power_flag == 0xFF)
			{				
				ShootMove(ShootTarget.position);
				StuckMoni();
			}
			else
			{
				if(RC_Ctl.mouse.press_r)
				{
					BurstMove(ShootTarget.speed);
					StuckMoni_Speed();
				}
				else
				{
					ShootMove(ShootTarget.position);
					StuckMoni();
				}
			}
		}
		else
		{
			BurstMove(-3000);
			inver_rotate_delay--;
		}
		
	}
	}
	else
	{
		friction_state_flag=0;
		if(remoteState == KEY_REMOTE_STATE||remoteState == NORMAL_REMOTE_STATE)
		{
			BurstMove(0);
		}
	}
}

float ssp=8;
float ssi=3.5;
float ssd=5;

void PidShootInit(void)
{
	PID_ControllerInit(&ShootPID.Position,120,500,5000,0.002);//120,500,10000,0.002
	PID_ControllerInit(&ShootPID.Speed,50,50,9800,0.002);//50,50,9800,0.002
	PID_ControllerInit(&MotorShoot,5,50,10000,0.002);//5,50,10000,0.002
	PID_SetGains(&ShootPID.Position,shoot_pp,shoot_pi,shoot_pd);
	PID_SetGains(&ShootPID.Speed,shoot_sp,shoot_si,shoot_sd);
	PID_SetGains(&MotorShoot,ssp,ssi,ssd);
}

void ShootMove(float SetPos)  //??????
{
	float NowPos = continuous_current_position_207;		//????????
	float NowSpeed=current_speed_207;
	

	PID_SetGains(&ShootPID.Position,shoot_pp,shoot_pi,shoot_pd);
	PID_SetGains(&ShootPID.Speed,shoot_sp,shoot_si,shoot_sd);

  PIDOut_Position_Shoot=PID_ControllerDriver(&ShootPID.Position,SetPos,NowPos);
	PIDOut_Speed_Shoot=PID_ControllerDriver(&ShootPID.Speed,PIDOut_Position_Shoot,NowSpeed);
	PIDOut_Whole_Shoot = PIDOut_Speed_Shoot;	

}

void BurstMove(float SetSpeed)
{
	float NowSpeed=current_speed_207;
	PIDOut_Whole_Shoot=PID_ControllerDriver(&ShootPID.Speed,SetSpeed,NowSpeed);
}


void StuckMoni(void)
{
	if((abs(current_speed_207)<1&&PIDOut_Whole_Shoot>8000))
	{
		stuck_count++;
	}
	if(stuck_count>300)
	{
		inver_rotate_delay=inver_rotate_delay_init;
		stuck_count=0;
	}
	
}

void StuckMoni_Speed(void)
{
	if((ShootTarget.speed>1000&&abs(current_speed_207<5)))
	{
		stuck_count++;
	}
	if(stuck_count>300)
	{
		inver_rotate_delay=inver_rotate_delay_init;
		stuck_count=0;
	}
}
/***************************************************************************************
 *Name     : Target
 *Function :
 *Input    :
 *Output   :
 *Description : 
****************************************************************************************/
u8 bullet=1;
int press_flag=0;
float ta_err=0;
int remain_bullet=0;

float target_pre=0;
float target_diff=0;

void Shoot_target_Cal(void)
{
	ta_err=fabs(ShootTarget.position-continuous_current_position_207);
	if(press_flag>0)
		press_flag--;
	if(remoteState == KEY_REMOTE_STATE)
	{
		if(frictionState == Friction_ON)
		{
			if(RC_Ctl.mouse.press_r&&remain_bullet>1)
			{
				ShootTarget.speed=5300;
				TargetRenew(1);

			}
			else if(RC_Ctl.mouse.press_l&&press_flag<=0&&ta_err<10&&remain_bullet>1)
			{				
				if(bullet>remain_bullet)
				{
					bullet=remain_bullet;
				}
				ShootTarget.position+=bullet*45;
				press_flag=200;
			}
			else if(remain_bullet<=1)
			{
				ShootTarget.speed=0;
				TargetRenew(1);
			}
		}
		else
		{
			ShootTarget.speed=0;
			TargetRenew(1);
		}
			
	}
	else if(remoteState == NORMAL_REMOTE_STATE)
	{
		if(frictionState == Friction_ON)
		{
			if(RC_Ex_Ctl.rc.s1==2)
				{
					ShootTarget.speed=2000;
					TargetRenew(1);
				}
				else
				{
					ShootTarget.speed=0;
					TargetRenew(1);
				}
		}
		else
		{
			ShootTarget.speed=0;
			TargetRenew(1);
		}
	}
	else if(remoteState == VIEW_STATE)
	{
		if(frictionState == Friction_ON)
		{
			if(big_power_flag == 0xFF)
			{
				//if(small_power_delay > SHOOT_DELAY)//1000ms 500ms
				
				if(RC_Ctl.mouse.press_l)
				{
					if(auto_begin == 1)
					{
						press_flag = 0;
					}
					auto_begin = 0;
				}
				if(RC_Ctl.mouse.press_r)
				{
					auto_begin = 1;
				}
				
				//if(/*RC_Ctl.mouse.press_l&&*/fram.yaw<0.2&&fram.pitch<0.5&& press_flag<=0)
				//if(auto_begin == 1 && press_flag<=0 && fram.yaw<0.2&&fram.pitch<0.5)
				if(auto_begin == 1 && press_flag<=0)
				{
						//small_power_Shoot_Interval = 60;
					  ShootTarget.position=continuous_current_position_207;
						ShootTarget.position+=1*45;//50?
					
					  target_diff=ShootTarget.position-target_pre;
					  target_pre=ShootTarget.position;
					  test_shootmotor_gothrought = continuous_current_position_207 - last_207;
					  last_207 = continuous_current_position_207;
					
						press_flag=700;

						//if_rotate_ok = 0;
						//small_power_delay = 0;
						
						//big_moveBegin = 0;
						//big_moveOver = 0;//ADD 2019.04.25
				}
				if(auto_begin == 0 && RC_Ctl.mouse.press_l && press_flag<=0)
				{
						ShootTarget.position=continuous_current_position_207;
						ShootTarget.position+=1*45;//50?
					
					  target_diff=ShootTarget.position-target_pre;
					  target_pre=ShootTarget.position;
					  test_shootmotor_gothrought = continuous_current_position_207 - last_207;
					  last_207 = continuous_current_position_207;
					
						press_flag=500;
				}
			}
			else
			{
				if(frictionState == Friction_ON)
				{
					if(RC_Ctl.mouse.press_r&&remain_bullet>1)
					{
						ShootTarget.speed=5300;
						TargetRenew(1);
					}
					//else if(RC_Ctl.mouse.press_l&&press_flag<=0&&ta_err<10&&remain_bullet>1)
					else if((RC_Ctl.mouse.press_l || ( ((KEY_PRESSED_OFFSET_C ) == KEY_PRESSED_OFFSET_C) 
						&& timeout_count < 1000 && (auto_aim_flag == 0xFF &&(fram.extra[0]==0x32))))&&press_flag<=0&&ta_err<10&&remain_bullet>1)
					{
						if(bullet>remain_bullet)
						{
							bullet=remain_bullet;
						}
						
						if(( ((KEY_PRESSED_OFFSET_C ) == KEY_PRESSED_OFFSET_C) && (auto_aim_flag == 0xFF &&(fram.extra[0]==0x32))))
						{
							if(remain_bullet < 3)
							{
								bullet=remain_bullet;
							}
							else
							{
								bullet = 3;
							}
						}

						
						ShootTarget.position+=bullet*45;
						press_flag=200;
					}
					else if(remain_bullet<=1)
					{
						ShootTarget.speed=0;
						TargetRenew(1);
					}
				}
				else
				{
					ShootTarget.speed=0;
					TargetRenew(1);
				}
			}
		}
	}
	else
	{
		TargetRenew(0);
	}
}


void TargetRenew(u8 flag)
{
	switch(flag)
	{
		case 0:
		{
			ShootTarget.position=continuous_current_position_207;
			ShootTarget.speed=current_speed_207;
		}break;
		case 1:
		{
			ShootTarget.position=continuous_current_position_207;
		}break;
		case 2:
		{
			ShootTarget.speed=current_speed_207;
		}break;
	}
}



/***************************************************************************************
 *Name     : Other Function
 *Function :
 *Input    :
 *Output   :
 *Description : 
****************************************************************************************/



void Friction_flag_moni(void)//invoke when Shoot_UP change
{
	if(frictionState==Friction_OFF)
	{
		friction_acce_flag_re=0;
		friction_dece_flag=0;
	}
	else 
	{
		if(remoteState == STANDBY_STATE ) 
		{
			friction_acce_flag_re=0;
			friction_dece_flag=0;
		}
		else if(remoteState == KEY_REMOTE_STATE ) 
		{
			if(Shoot_UP_Pre<Shoot_UP)friction_acce_flag_re=1;
			else if(Shoot_UP_Pre>Shoot_UP)friction_dece_flag=1;
		}
		else if(remoteState == NORMAL_REMOTE_STATE ) 
		{
			if(Shoot_UP_Pre<Shoot_UP)friction_acce_flag_re=1;
			else if(Shoot_UP_Pre>Shoot_UP)friction_dece_flag=1;
		}
		else if(remoteState == VIEW_STATE ) 
		{
			if(Shoot_UP_Pre<Shoot_UP)friction_acce_flag_re=1;
			else if(Shoot_UP_Pre>Shoot_UP)friction_dece_flag=1;
		}
		else if(remoteState == ERROR_STATE ) 
		{
			friction_acce_flag_re=0;
			friction_dece_flag=0;
		}
		else if(remoteState == PREPARE_STATE ) 
		{
			friction_acce_flag_re=0;
			friction_dece_flag=0;
		}
	}
	if(frictionState==Friction_ON&&friction_speed_temp<Shoot_UP)friction_acce_flag_re=1;
	if(frictionState==Friction_ON&&friction_speed_temp>Shoot_UP)friction_dece_flag=1;
	if(frictionState==Friction_OFF)
	{
			friction_acce_flag_re=0;
			friction_dece_flag=0;
	}
}

void Shoot_Para_Moni(void)
{
	if(raging_mode==1)
	{
		heat_limite=9999;
	}
	else if(ext_game_robot_state.robot_level == 1 )
	{
		heat_limite=240;
	}
	else if(ext_game_robot_state.robot_level == 2 )
	{
		heat_limite=360;
	}
	else if(ext_game_robot_state.robot_level == 3 )
	{
		heat_limite=480;
	}

	
	if(((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_R )==KEY_PRESSED_OFFSET_R) || (auto_aim_flag == 0xFF  &&(fram.extra[0]==111)))
	{
		bullet_clear_count++;
		if(bullet_added_flag==0&&bullet<5)
		{
			bullet=Running_Fire_NUM;
			bullet_added_flag=1;
			bullet_add_delay=bullet_add_delay_init;
		}
		
		if(bullet_clear_count>=300)
		{ 
			bullet=1;
			bullet_clear_count=0;
		}
	}
	else
	{
		if(bullet_added_flag==1)
		{
			bullet_add_delay--;
		}
		if(bullet_add_delay<=0)
		{
			bullet_added_flag=0;
			bullet_clear_count=0;
		}
	}
	
	if(shoot_speed_change_flag_count>0)
		shoot_speed_change_flag_count--;
	if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL )==KEY_PRESSED_OFFSET_CTRL&&shoot_speed_change_flag_count<=0)
	{
		if(shoot_up_speed_flag==0)
			shoot_up_speed_flag=1;
		else
			shoot_up_speed_flag=0;
		shoot_speed_change_flag_count=200;
	}//press to switch friction speed
//	if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL )==KEY_PRESSED_OFFSET_CTRL)shoot_up_speed_flag=1;
//	else shoot_up_speed_flag=0;//hold to switch friction speed
	
	
	
	if(big_power_flag == 0xFF)
	{
		Shoot_UP=Shoot_UP_HIGH;
		Friction_flag_moni();
		remain_bullet=(heat_limite-ext_power_heat_data.shooter_heat0)/30.0f;
	}
	else
	{
		if(shoot_up_speed_flag==1)
		{
			Shoot_UP=Shoot_UP_HIGH;
			Friction_flag_moni();
			remain_bullet=(heat_limite-ext_power_heat_data.shooter_heat0)/30.0f;
		}
		else
		{
			Shoot_UP=Shoot_UP_LOW;
      Friction_flag_moni();
			remain_bullet=(heat_limite-ext_power_heat_data.shooter_heat0)/20.0f;
		}
	}
	

	
}


void Cover_Moni(void)
{
	if(RC_Ex_Ctl.rc.s1==2)
	{
		TIM_SetCompare1(TIM9, COVER_OPEN);
	}
	else
	{
		TIM_SetCompare1(TIM9, COVER_CLOSE);

	}

	
}
float friction_current_speed(void)
{
	if( Friction_CH1.speed[0]<=Friction_CH2.speed[0])return (float)Friction_CH1.speed[0];
	else return (float)Friction_CH2.speed[0];
}
/***************************************************************************************
 *Name     : 
 *Function : ???????? ???timer.c
****************************************************************************************/
float COMPENSATE  = -0.f;
float MAFilter_Threshold = 50.f;

void dmaFrictionUpdata(PWMFriction_Type* friction) 
	{
    uint8_t overflow = 0;
    uint8_t size=sizeof(friction->counters)/sizeof(friction->counters[0])-1;
    
    friction->stopping = 0;
    for(uint8_t i=1; i<size; i++) {
        overflow += (friction->counters[i]<friction->counters[i-1]);
    }
    friction->counter = (friction->counters[size-1] +
        overflow * (TIM4->ARR+1) - friction->counters[0])/(size-1);
    /* Using limited moving average */
    #ifdef USING_FRICTION_FILTER
        MovingAverageFilter_f32((float*)friction->speed, 
                sizeof(friction->speed)/sizeof(friction->speed[0]), 
                (friction->counter>0)? (619195.05f/(2*friction->counter)-33.356f+COMPENSATE):0, 
                MAFilter_Threshold);
    #else
        friction->speed[0] = (friction->counter>0)? (619195.05f/(2*friction->counter)-33.356f+COMPENSATE):0;
    #endif
    friction->changed = 1;
//        friction->counter = counter;
}

float delta_lim = 2;
float dspd_gain = 1.6f, dspd_gain2 = 0.0f;

void looperUpdateFriction(PWMFriction_Type* friction)
	{
    friction->output[2] = friction->output[1];
    friction->output[1] = friction->output[0];
    
    float d_spd = friction->speed[0] - friction->speed[1];
    float d2_spd = friction->speed[1] - friction->speed[2];
    if(friction->target < 10) {
        friction->output[0] = 0;
        friction->pid.sum_error = 0;
    }
    else if(d_spd < 0) {
        friction->output[0] += fabs(d_spd) * dspd_gain + fabs(d2_spd)*dspd_gain2;
    }
    else {
        friction->output[0] = friction->target;
    }
    
    friction->speed[1] = friction->speed[0];
    friction->speed[2] = friction->speed[1];

}

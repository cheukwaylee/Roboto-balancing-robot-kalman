/************************************************************
 *File		:	ShootControl.c
 *Author	:  @YUYENCHIA ,jiafish@outlook.com	
 *Version	: V1.0
 *Update	: 2018.12.11
 *Description: 	
 ************************************************************/

#include "main.h"

#define Friction_OFF 0
#define Friction_ON  1
#define Shoot_speed -2000//-2000
#define Friction_speed 8000//100

typedef struct _PIDConfig{
	float pp;
	float pi;
	float pd;
	float sp;
	float si;
	float sd;
} PIDConfig;

PIDConfig PID_Friction={0.0,0.0,0.0,20.0,0.0,1.0};
PIDConfig PID_Shoot = {100.0,2.0,0.0,16.0,0.0,0.0};

float PIDOut_Friction1=0;
float PIDOut_Friction2=0;
float PIDOut_Position_Shoot,PIDOut_Speed_Shoot,PIDOut_Whole_Shoot;
ShootPID_Type ShootPID;
ShootPID_Type FrictionPID1;
ShootPID_Type FrictionPID2;

int16_t frictionState;
int pre_friction_key = 0;

/**************************************************************************************/
// Unused 
PWMFriction_Type    Friction_CH1;
PWMFriction_Type    Friction_CH2;
u8 shoot_up_speed_flag=0;
u8 friction_state_flag=0;
u8 bullet=3;
int remain_bullet=0;
/**************************************************************************************/
void ShootControlInit(void)
{
	PidShootInit();	
	ShootStop();
}

void ShootStop(void)				
{
	frictionState=Friction_OFF;
	CAN2_Cmd_Friction_SHOOT(0,0,0);
	PIDOut_Friction1=0;
	PIDOut_Friction2=0;
	PIDOut_Whole_Shoot=0;
}

void ShootControlLoop(void)
{

	if(remoteState == PREPARE_STATE)
	{
		ShootStop();
	}
	else if(remoteState == NORMAL_REMOTE_STATE)
	{
		Friction_Moni_normal();
		
		Shoot_target_Cal();

		Shooter_Moni();
		
		CAN2_Cmd_Friction_SHOOT(PIDOut_Friction1,PIDOut_Friction2,PIDOut_Whole_Shoot);
	}
	else{
		ShootStop();
		
	}

	
}


void Friction_Moni_normal(void)
{

	if (pre_friction_key==1 && RC_Ctl.rc.s1==3 && frictionState == Friction_OFF){
		frictionState = Friction_ON;
	}
	else if(pre_friction_key==1 && RC_Ctl.rc.s1==3 &&frictionState == Friction_ON){
		frictionState = Friction_OFF;
	}
	
	if (frictionState == Friction_ON){
		BurstMove_Friction(Friction_speed);
	}else{
		BurstMove_Friction(0);
	}
	

	if (frictionState == Friction_ON && RC_Ctl.rc.s1==2){
		BurstMove(Shoot_speed);
	}else
	{
		BurstMove(0);
	}

	
	
	pre_friction_key=RC_Ctl.rc.s1;

}

void BurstMove_Friction(float SetSpeed)
{
	float NowSpeed_205=can2_current_speed_205;
	float NowSpeed_206=can2_current_speed_206;
	PIDOut_Friction1=PID_ControllerDriver(&FrictionPID1.Speed,-SetSpeed,NowSpeed_205);
	PIDOut_Friction2=PID_ControllerDriver(&FrictionPID2.Speed,SetSpeed,NowSpeed_206);
	
	
}

void BurstMove(float SetSpeed)
{
	float NowSpeed=current_speed_207;
	PIDOut_Whole_Shoot=PID_ControllerDriver(&ShootPID.Speed,SetSpeed,NowSpeed);
}

void PidShootInit(void)
{
	PID_ControllerInit(&ShootPID.Position,120,500,5000,0.002);//120,500,10000,0.002
	PID_ControllerInit(&ShootPID.Speed,50,50,9800,0.002);//50,50,9800,0.002
	PID_SetGains(&ShootPID.Position,PID_Shoot.pp,PID_Shoot.pi,PID_Shoot.pd);
	PID_SetGains(&ShootPID.Speed,PID_Shoot.sp,PID_Shoot.si,PID_Shoot.sd);

	//PID_ControllerInit(&FrictionPID1.Position,120,500,5000,0.002);
	PID_ControllerInit(&FrictionPID1.Speed,50,200,12000,0.002);
	//PID_SetGains(&FrictionPID1.Position,PID_Friction.pp,PID_Friction.pi,PID_Friction.pd);
	PID_SetGains(&FrictionPID1.Speed,PID_Friction.sp,PID_Friction.si,PID_Friction.sd);
	
	//PID_ControllerInit(&FrictionPID2.Position,120,500,5000,0.002);
	PID_ControllerInit(&FrictionPID2.Speed,50,200,12000,0.002);
	//PID_SetGains(&FrictionPID2.Position,PID_Friction.pp,PID_Friction.pi,PID_Friction.pd);
	PID_SetGains(&FrictionPID2.Speed,PID_Friction.sp,PID_Friction.si,PID_Friction.sd);
}
/****************************************************************************************/
// Unused functions
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
void Shoot_target_Cal(void)
{
	
}

void Shooter_Moni(void){}
	



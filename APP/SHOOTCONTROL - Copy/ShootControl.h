#ifndef _SHOOTCONTROL_H_
#define _SHOOTCONTROL_H_

#include "main.h"

#define USING_FRICTION_FILTER
#define Running_Fire_NUM 3
extern u8 bullet;
extern u8 friction_state_flag;
extern int remain_bullet;
extern u8 shoot_up_speed_flag;
typedef struct {
    uint8_t             changed:1;
    uint8_t             stopping:4;
    volatile float      speed[3];
    uint16_t            target;
    volatile int16_t    output[3];
    PID_Type            pid;
    volatile uint32_t   counters[3];
    uint32_t            counter;
} PWMFriction_Type;

extern PWMFriction_Type    Friction_CH1;
extern PWMFriction_Type    Friction_CH2;

typedef struct
	{
	  PID_Type Speed;
		PID_Type Position;
	}ShootPID_Type;
typedef struct
	{
	  float speed;
		long long int position;
	}Shoot_Target;
	
	
	
	
	void ShootControlInit(void);
	void ShootStop(void);
	void ShootControlLoop(void);
	
	void Friction_Moni(void);
	void Friction_flag_moni(void);
	void FrictionJudge(void);
	void FrictionControl(void);
	void Friction_Init(void);
	void Friction_Looper(uint32_t target);
	uint32_t Fricton_restart(void);
	
	void Shooter_Moni(void);
	void PidShootInit(void);
	void ShootMove(float SetPos);
	void BurstMove(float SetSpeed);
	void StuckMoni(void);
	void StuckMoni_Speed(void);
	
	void PWM_self_adjust(void);
	
	void Shoot_target_Cal(void);
	void TargetRenew(u8 flag);
	
	void Shoot_Para_Moni(void);
	void Cover_Moni(void);
	float friction_current_speed(void);
	
	extern float PIDOut_Whole_Shoot;
	extern Shoot_Target ShootTarget;
	
	
	void dmaFrictionUpdata(PWMFriction_Type* friction) ;
	void looperUpdateFriction(PWMFriction_Type* friction);
	void Friction_Moni_normal(void);
	void BurstMove_Friction(float SetSpeed);

#endif

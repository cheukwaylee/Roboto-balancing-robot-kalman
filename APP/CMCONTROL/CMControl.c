/****************************************************************************************************
 *File		:	CMControl.c
 *Author	:  @YangTianhao ,490999282@qq.com，@TangJiaxin ,tjx1024@126.com
 *Version	: V1.0
 *Update	: 2017.12.11
 *Description: 	Control Chassis Motors.
								CMControlLoop() shows the way to control the motion of chassis in different states.
                Use PID to optimize Chassis motor control.
 *****************************************************************************************************/

#include "main.h"

int16_t CMFollowVal=0; 			    //底盘跟随值
int16_t speedA_final,speedB_final,speedC_final,speedD_final;
//PID_Struct CMPid,CMFollowPid,CMFollowPid_speed;                 //底盘运动pid、跟随pid
float step=100,keymove_x=0,keymove_y=0,m=0,n=0,q=0;           //step-速度变化率  x-x轴方向变化值 y-y速度变化值;




u8 quick_spin_flag=0;
int16_t CMFollowVal_QUICK=0;


PID_Type SPFOLLOW,SPCHIASSISA,SPCHIASSISB,SPCHIASSISC,SPCHIASSISD,SPFOLLOW_SPEED;


float max_output_speed=600;

float singl_max=4000;
float speed_step=500;
float output_current_sum=0;

float spin_y=0;
float spin_x=0;

u8 climb_mode_flag=0;


float cm_normal_p=2.0;
float cm_normal_i=0.0;
float cm_normal_d=0.0;

float cm_climb_p=8.5;
float cm_climb_i=20;
float cm_climb_d=0.0;

float rotate_speed=250;
int rotate_speed_dir=1;
float rotate_change_para=1;
int rotate_counter=0;
int rotate_counter_100ms=0;

/*************balance chiassis********/
float pp=180.0;
float pi=10.0;
float pd=5.0;
float sp=50;
float si=1.0;
float sd=3.0;

// entries of the dynamics matrix (A)
float A33 = 0;
float A34 = 1;
float A43 = 0;
float A44 = 36.9312;

// entries of the inputs matrix (B)
float B3 = 0;
float B4 = 0.9402;

// entries of the outputs matrix (C)
float C33 = 1;
float C34 = 0;
float C43 = 0;
float C44 = 1;

// entries of the Kalman filter matrix (Kf)
float Kf33 = 0.3399;
float Kf34 = 1.9922;
float Kf43 = 1.9922;
float Kf44 = 11.9699;

// entries of the matrix gave by multiplying the Kalman filter matrix (Kf) and the outputs matrix (C)
float KfC33 = 0.3399;
float KfC34 = 1.9922;
float KfC43 = 1.9922;
float KfC44 = 11.9699;

// entries of the LQR gain (K)
float K3 = 79.1914;
float K4 = 14.0874;


GimbalPID ChassiPitchPID;
float CMbalanceVal=0;
/*-------------  底盘控制循环  -------------*/
void CMControlLoop(void)
{

    CM_Switch_Moni();
    if(remoteState == PREPARE_STATE)
    {
        CMControlOut(0,0,0,0);
    }
    else if(remoteState == NORMAL_REMOTE_STATE)
    {
//        CMFollowVal = followValCal(0);
//        move(RC_Ex_Ctl.rc.ch0/10.24,RC_Ex_Ctl.rc.ch1/5.12,CMFollowVal/*0.5f*(RC_Ex_Ctl.rc.ch2)*/);
			//CMbalanceVal = caculate_balance(0.15*57.3);
			//move_balance(RC_Ex_Ctl.rc.ch1/5.12,0.5f*(RC_Ex_Ctl.rc.ch2),CMbalanceVal);
			move_balance(0,0,CMbalanceVal);
			kalman_filter_update(input_to_wheels, Pitch, Pitch_gyro);	// update the estimation of the balancing robot state
    }
    else if(remoteState == STANDBY_STATE )
    {
        CMStop();
    }
    else if(remoteState == ERROR_STATE )
    {
        CMStop();
    }
    else if(remoteState == KEY_REMOTE_STATE )
    {
				CMStop();
    }
    else if(remoteState == VIEW_STATE )
    {
				CMStop();
    }


}


float NowPosition_b = 0;
float caculate_balance(float Setposition){
	// Setposition is measured in rad
	float balanceVal = 0;
	float NowPosition = Pitch*57.3;
	
	float NowSpeed = Pitch_gyro*57.3;
//	if (abs(NowPosition)<20){
//		PID_SetGains(&ChassiPitchPID.Position,pp*(1-abs(NowPosition)/20.0),pi,pd);
//	}else{
//		return 0;
//	}
	balanceVal = PID_ControllerDriver(&ChassiPitchPID.Position,Setposition,NowPosition);
//	balanceVal=PID_ControllerDriver(&ChassiPitchPID.Speed,balanceVal/5.0f,NowSpeed);
	return balanceVal;
//	history[history_pos]=PID_ControllerDriver(&ChassiPitchPID.Speed,balanceVal/5.0f,NowSpeed);
//	if (history_pos<49){
//		history_pos+=1;
//	}
//	else{
//		history_pos=0;
//	}
//	NowPosition_b=0;
//	for (int16_t i=0;i<50;i++){
//		//NowPosition +=history[i];
//		NowPosition_b +=history[i];
//	}
//	balanceVal = NowPosition_b/50;
//	return balanceVal;
		
}
float out_speedL=0;
float out_speedR=0;
int16_t speedL=0;
int16_t speedR=0;
float input_to_wheels=0;	// input to the wheels of balancing robot (to be used in Kalman filter)
float history[10]={0,0,0,0,0,0,0,0,0,0};
int16_t history_pos=0 ;
void move_balance(int16_t speedY, int16_t rad,int16_t balance){
//	float max_speed=0;
//	speedL = speedY + balance + rad;
//	speedR = -speedY - balance + rad;
//	
//	if(fabs(speedL)>max_speed)
//        max_speed=fabs(speedL);
//	if(fabs(speedR)>max_speed)
//			max_speed=fabs(speedR);
//	
//	if(max_speed>max_output_speed)
//	{
//		speedL*=max_output_speed/max_speed;
//		speedR*=max_output_speed/max_speed;
//	}
//	float now_Pitch =0;
//	if (abs(Pitch)>0.01){
//		now_Pitch=Pitch;
//	}
	
	speedL = 1*((0.25-Pitch_estim)*57.3*K3+(0-Pitch_gyro_estim)*57.3*K4);
	speedR = -speedL;

	CMControlOut(speedL,speedR,0,0);
	//CAN1_Send_Bottom(speedL,speedR,0,0); //to send the control signals directly to the wheels motors
}

/*------- estimates the angular position and velocity of the robot by its inputs and outputs -------*/
void kalman_filter_update(int16_t u, float y3, float y4)
{
		// equation for state estimation: x_hat(k+1) = (A - Kf*C)*x_hat(k) + B*u(k) + Kf*y(k)
		Pitch_estim =
			(A33 - KfC33)*Pitch_estim + (A34 - KfC34)*Pitch_gyro_estim + B3*u + Kf33*y3 + Kf34*y4;
		Pitch_gyro_estim =
			(A43 - KfC43)*Pitch_estim + (A44 - KfC44)*Pitch_gyro_estim + B4*u + Kf43*y3 + Kf44*y4;
}

/*-------------  底盘停止  -------------*/
void CMStop(void)
{
    CAN1_Send_Bottom(0,0,0,0);
}


/***************************************************************************************
 *Name     : move
 *Function ：计算底盘电机速度给定值 由遥控器控制。同时有各个轮子的速度限制
 *Input    ：speedX, speedY, rad
 *Output   ：无
 *Description :改变了全局变量	speedA, speedB, speedC, speedD
****************************************************************************************/
void move(int16_t speedX, int16_t speedY, int16_t rad)
{
    float max_speed=0;
    speedX *= gears_speedXYZ;
    speedY *= gears_speedXYZ;
    rad *= gears_speedRAD;


//		if(fabs(speedY)>6000&&fabs(rad)<100)
//		{
//			rad*=10;
//		}

    int16_t speedA = ( speedX + speedY + rad);
    int16_t speedB = ( speedX - speedY + rad);
    int16_t speedC = (-speedX - speedY + rad);
    int16_t speedD = (-speedX + speedY + rad);

    if(fabs(speedA)>max_speed)
        max_speed=fabs(speedA);
    if(fabs(speedB)>max_speed)
        max_speed=fabs(speedB);
    if(fabs(speedC)>max_speed)
        max_speed=fabs(speedC);
    if(fabs(speedD)>max_speed)
        max_speed=fabs(speedD);

    if(max_speed>max_output_speed)
    {
        speedA*=max_output_speed/max_speed;
        speedB*=max_output_speed/max_speed;
        speedC*=max_output_speed/max_speed;
        speedD*=max_output_speed/max_speed;
    }

    CMControlOut(speedA,speedB,speedC,speedD);
}


void key_move(int16_t speedX, int16_t speedY, int16_t rad)
{
    float max_speed=0;
    rad *= gears_speedRAD;

    if(fabs(speedY)>5000&&fabs(rad)<50&&fabs(rad)>5)
    {
        rad*=10;
    }
    int16_t speedA = ( speedX + speedY + rad);
    int16_t speedB = ( speedX - speedY + rad);
    int16_t speedC = (-speedX - speedY+ rad);
    int16_t speedD = (-speedX + speedY + rad);

    if(fabs(speedA)>max_speed)
        max_speed=fabs(speedA);
    if(fabs(speedB)>max_speed)
        max_speed=fabs(speedB);
    if(fabs(speedC)>max_speed)
        max_speed=fabs(speedC);
    if(fabs(speedD)>max_speed)
        max_speed=fabs(speedD);

    if(max_speed>max_output_speed)
    {
        speedA*=max_output_speed/max_speed;
        speedB*=max_output_speed/max_speed;
        speedC*=max_output_speed/max_speed;
        speedD*=max_output_speed/max_speed;
    }

    CMControlOut(speedA,speedB,speedC,speedD);
}



/***************************************************************************************
 *Name     : CMControlOut
 *Function ：底盘电机速度输出，经过PID控制器，发送给电机驱动器，同时有各个轮子的速度限制
 *Input    ：speedA,speedB,speedC,speedD
 *Output   ：无
 *Description : 底盘电机速度环和幅值函数
****************************************************************************************/
int16_t out_a=0;
float lec_numA=3000;
float lec_numB=3000;
float lec_numC=3000;
float lec_numD=3000;

int16_t pre_current_A=0;
int16_t pre_current_B=0;
int16_t pre_current_C=0;
int16_t pre_current_D=0;
int16_t current_step=600;//80;
u8 super_cap_flag=0;
void CMControlOut(int16_t spa , int16_t spb ,int16_t spc ,int16_t spd )
{

    float current_sum=0;
    int max_now_speed=0;
	
    float speedA = PID_ControllerDriver(&SPCHIASSISA,spa,current_cm_201);
    float speedB = PID_ControllerDriver(&SPCHIASSISB,spb,current_cm_202);
    float speedC = PID_ControllerDriver(&SPCHIASSISC,spc,current_cm_203);
    float speedD = PID_ControllerDriver(&SPCHIASSISD,spd,current_cm_204);

		input_to_wheels = speedA;	// recovering the input sent to wheels, so that it can be used in the Kalman filter
	
   // if(cap_receive.state_module==Cap_Run&&cap_receive.state_cap==Cap_Boost&&cap_receive.CapVol>11800)
    if(1)
    {
        CAN1_Send_Bottom(speedA,speedB,speedC,speedD);
        super_cap_flag=0;
    }
    else
    {
        super_cap_flag=1;
        if(abs(current_cm_201)>max_now_speed)
            max_now_speed=abs(current_cm_201);
        if(abs(current_cm_202)>max_now_speed)
            max_now_speed=abs(current_cm_202);
        if(abs(current_cm_203)>max_now_speed)
            max_now_speed=abs(current_cm_203);
        if(abs(current_cm_204)>max_now_speed)
            max_now_speed=abs(current_cm_204);


        if(abs(speedA-pre_current_A)>current_step)
        {
            if(speedA-pre_current_A>current_step)
                speedA=pre_current_A+current_step;
            else if(speedA-pre_current_A<-current_step)
                speedA=pre_current_A-current_step;
        }

        if(abs(speedB-pre_current_B)>current_step)
        {
            if(speedB-pre_current_B>current_step)
                speedB=pre_current_B+current_step;
            else if(speedB-pre_current_B<-current_step)
                speedB=pre_current_B-current_step;
        }
        if(abs(speedC-pre_current_C)>current_step)
        {
            if(speedC-pre_current_C>current_step)
                speedC=pre_current_C+current_step;
            else if(speedC-pre_current_C<-current_step)
                speedC=pre_current_C-current_step;
        }
        if(abs(speedD-pre_current_D)>current_step)
        {
            if(speedD-pre_current_D>current_step)
                speedD=pre_current_D+current_step;
            else if(speedD-pre_current_D<-current_step)
                speedD=pre_current_D-current_step;
        }
        current_sum=fabs(speedA)+fabs(speedB)+fabs(speedC)+fabs(speedD);



        if(current_sum > max_output_current)
        {
            speedA*=(max_output_current)/current_sum;
            speedB*=(max_output_current)/current_sum;
            speedC*=(max_output_current)/current_sum;
            speedD*=(max_output_current)/current_sum;
        }
        CAN1_Send_Bottom(speedA,speedB,speedC,speedD);
    }

    output_current_sum=abs(speedA)+abs(speedB)+abs(speedC)+abs(speedD);
    pre_current_A=speedA;
    pre_current_B=speedB;
    pre_current_C=speedC;
    pre_current_D=speedD;


}


/**********  底盘电机幅值限制  **********/
float CMSpeedLegalize(float MotorCurrent , float limit)
{
    return MotorCurrent<-limit?-limit:(MotorCurrent>limit?limit:MotorCurrent);
}


/***************************************************************************************
 *Name     : followValCal
 *Function ：底盘电机跟随计算
 *Input    ：Setposition,Yaw轴电机码盘跟随值
 *Output   ：底盘跟随量
 *Description : 底盘电机跟随量计算函数
****************************************************************************************/
int16_t followValCal(float Setposition)
{
    int16_t followVal = 0;
    float NowPosition = position_yaw_relative;
    followVal=PID_ControllerDriver(&SPFOLLOW,Setposition,NowPosition);
//	float followVal_speed=PID_ControllerDriver(&SPFOLLOW_SPEED,followVal/20.0,yaw_speed);
//	followVal = CMSpeedLegalize(followVal_speed,1200);
    //跟随量最小值，角度过小不跟随
    //if(abs(followVal) < followVal_limit) followVal = 0;

    return followVal;
}



/*-------------  底盘电机速度PID和跟随PID初始化  -------------*/



/*底盘初始化*/
void CMControlInit(void)
{
    PID_ControllerInit(&SPFOLLOW,50,150,1100,0.01);
    SPFOLLOW.intergration_separation = 100;
		int16_t max_out = 5000;
    PID_ControllerInit(&SPCHIASSISA,20,20,max_out,0.01);
    PID_ControllerInit(&SPCHIASSISB,20,20,max_out,0.01);
    PID_ControllerInit(&SPCHIASSISC,20,20,max_out,0.01);
    PID_ControllerInit(&SPCHIASSISD,20,20,max_out,0.01);
    PID_ControllerInit(&SPFOLLOW_SPEED,20,20,1500,0.01);
    PID_SetGains(&SPFOLLOW, 0.29,0.3,0.004);// 0.28,0.3,0.8
    PID_SetGains(&SPFOLLOW_SPEED,3,0.5,0.1);
    CM_Normal_PID();
		
		PID_ControllerInit(&ChassiPitchPID.Position,1000,500,8000,0.01);
    PID_ControllerInit(&ChassiPitchPID.Speed,50,50,8000,0.01);
		PID_SetGains(&ChassiPitchPID.Position,pp,pi,pd);
    PID_SetGains(&ChassiPitchPID.Speed,sp,si,sd);
	
}

void CM_Normal_PID(void)
{
    PID_SetGains(&SPCHIASSISA,cm_normal_p,cm_normal_i,cm_normal_d);
    PID_SetGains(&SPCHIASSISB,cm_normal_p,cm_normal_i,cm_normal_d);
    PID_SetGains(&SPCHIASSISC,cm_normal_p,cm_normal_i,cm_normal_d);
    PID_SetGains(&SPCHIASSISD,cm_normal_p,cm_normal_i,cm_normal_d);
	PID_SetGains(&SPFOLLOW, 0.29,0.3,0.004);// 0.28,0.3,0.8
}

void CM_Climb_PID(void)
{
    PID_SetGains(&SPCHIASSISA,cm_climb_p,cm_climb_i,cm_climb_d);
    PID_SetGains(&SPCHIASSISB,cm_climb_p,cm_climb_i,cm_climb_d);
    PID_SetGains(&SPCHIASSISC,cm_climb_p,cm_climb_i,cm_climb_d);
    PID_SetGains(&SPCHIASSISD,cm_climb_p,cm_climb_i,cm_climb_d);
	PID_SetGains(&SPFOLLOW, 0.23,0.3,0.002);// 0.28,0.3,0.8
}


void keyboardmove(uint16_t keyboardvalue,uint16_t xlimit,uint16_t ylimit)
{

    // W and S
    switch(keyboardvalue & (KEY_PRESSED_OFFSET_W|KEY_PRESSED_OFFSET_S))
    {
    case ( KEY_PRESSED_OFFSET_W):
        m = 0;
        if(keymove_y>0)
            keymove_y += 0.8f*step;
        else
            keymove_y += step*2;
        break;
    case ( KEY_PRESSED_OFFSET_S):
        m = 0;
        if(keymove_y<0)
            keymove_y -= 0.8f*step;
        else
            keymove_y -= step*2;
        break;
    default:
        m++;
        if(m>1)
        {
            if(keymove_y>3*step) {
                keymove_y=keymove_y-3*step;
            }
            else if(keymove_y<-3*step) {
                keymove_y=keymove_y+3*step;
            }
            else {
                keymove_y = 0;
            }
        }
        break;
    }

    //  A and D  stand for X axis
    switch(keyboardvalue & (KEY_PRESSED_OFFSET_A | KEY_PRESSED_OFFSET_D))
    {
    case ( KEY_PRESSED_OFFSET_A):
        n = 0;
        if(keymove_x<0)
            keymove_x -= 3*step;
        else
            keymove_x -= 3*step;
        break;
    case ( KEY_PRESSED_OFFSET_D):
        n = 0;
        if(keymove_x>0)    //正在向右
            keymove_x += 3*step;           //加步长
        else
            keymove_x += 3*step;
        break;

    default:
        n++;
        if(n>1)
        {
            if(keymove_x>3*step) {
                keymove_x=keymove_x-3*step;
            }
            else if(keymove_x<-3*step) {
                keymove_x=keymove_x+3*step;
            }
            else {
                keymove_x = 0;
            }
        }
        break;
    }
    keymove_x = keymove_x>xlimit?xlimit:(keymove_x<-xlimit?-xlimit:keymove_x);  //限速
    keymove_y = keymove_y>ylimit?ylimit:(keymove_y<-ylimit?-ylimit:keymove_y);

    if(keymove_x<5&&keymove_x>-5) keymove_x = 0;
    if(keymove_y<5&&keymove_y>-5) keymove_y = 0;	 //减少爬行

}

u8 cm_switch_delay=0;


int rotate_shift_count=0;
void CM_Switch_Moni(void)
{
    if(cm_switch_delay>0)
        cm_switch_delay--;


    if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_X )==KEY_PRESSED_OFFSET_X&&cm_switch_delay<=0)
    {
        if(climb_mode_flag==1)
            climb_mode_flag=0;
        else
            climb_mode_flag=1;
        cm_switch_delay=50;
    }

    if(	rotate_shift_count>-1)
    {
        rotate_shift_count--;
    }

    if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT )==KEY_PRESSED_OFFSET_SHIFT)
    {
        rotate_shift_count=50;
    }
    if(rotate_shift_count==0&&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT )!=KEY_PRESSED_OFFSET_SHIFT)
    {
        rotate_speed_dir=-rotate_speed_dir;
    }


}








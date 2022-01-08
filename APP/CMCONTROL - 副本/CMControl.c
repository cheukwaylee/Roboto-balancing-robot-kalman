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

int16_t CMFollowVal=0,CMSwingVal=0; 			    //底盘跟随值
int16_t speedA,speedB,speedC,speedD;          //对应机器顺时针方向1、2、3、4号轮子,即左前、右前、右后、右左  
PID_Struct CMPid,CMFollowPid;                 //底盘运动pid、跟随pid
float step=6,keymove_x=0,keymove_y=0,m=0,n=0,q=0;           //step-速度变化率  x-x轴方向变化值 y-y速度变化值; 
int16_t xlimit = XLIMIT_OFFSET,ylimit = YLIMIT_OFFSET;
float last_ele_dec = 0;
float now_ele_dec = 0;
float speed_limite=6000;
//follow
int16_t follow_val_temp = 0;

//swing
u8 flag_swing = 0;
int16_t swing_count = 0,swing_dir = 1;   //dir = 1 : +    dir = -1 : -
u8 quick_spin_flag=0;
int16_t CMFollowVal_QUICK=0;


PID_Type SPFOLLOW,SPCHIASSISA,SPCHIASSISB,SPCHIASSISC,SPCHIASSISD;




/*-------------  底盘控制循环  -------------*/
void CMControlLoop(void)
{
	//PidValControl(&CMFollowPid);
	if(remoteState == PREPARE_STATE)
	{
		  CMControlOut(0,0,0,0);
	}
	else if(remoteState == NORMAL_REMOTE_STATE)   
	{
		  CMFollowVal = followValCal(0);
		  move(RC_Ex_Ctl.rc.ch0,RC_Ex_Ctl.rc.ch1,CMFollowVal );
	
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
		 if(flag_swing == 1)
		 {
				
				SwingFollowValCal();                      //如果按下Q或E，CMSwingVal变化，扭腰
				CMFollowVal = followValCal(CMSwingVal);   //底盘跟随PID计算
			  keyboardmove(RC_Ex_Ctl.key.v);            //检测键盘是否有键按下
				move(keymove_x,keymove_y,CMFollowVal);	  //底盘运动函数
		 }
		 else
		 {
			 if(abs(position_yaw_relative)<5)
			 {
				keyboardmove(RC_Ex_Ctl.key.v);            //检测键盘是否有键按下
				CMFollowVal = 0;   //底盘跟随PID计算
			  swing_dir		= 1;
			  swing_count = 0;
			  CMSwingVal 	= 0;
			 }
			 else
			 {
				 keyboardmove(RC_Ex_Ctl.key.v);            //检测键盘是否有键按下
				CMFollowVal = followValCal(CMSwingVal);   //底盘跟随PID计算
			  swing_dir		= 1;
			  swing_count = 0;
			  CMSwingVal 	= 0;
			 }
			  
				if(quick_spin_flag==0)
				{	
					move(keymove_x,keymove_y,CMFollowVal);	  //底盘运动函数
				}
				else
				{
					move(keymove_x,keymove_y,CMFollowVal_QUICK);
				}
		 }
	}
		else if(remoteState == VIEW_STATE )
	{		  
			if(flag_swing == 1)
		 {
				
				SwingFollowValCal();                      //如果按下Q或E，CMSwingVal变化，扭腰
				CMFollowVal = followValCal(CMSwingVal);   //底盘跟随PID计算
			  keyboardmove(RC_Ex_Ctl.key.v);            //检测键盘是否有键按下
				move(keymove_x,keymove_y,CMFollowVal);	  //底盘运动函数
		 }
		 else
		 {
			 if(big_power_flag == 0xFF && auto_aim_flag == 0)
			 {
				 keyboardmove(RC_Ex_Ctl.key.v); 
				 swing_dir		= 1;
				 swing_count = 0;
				 CMSwingVal 	= 0;
				 move(keymove_x,keymove_y,RC_Ex_Ctl.mouse.x*0.1f);	  //底盘运动函数
			 }
			 else
			 {
					keyboardmove(RC_Ex_Ctl.key.v);            //检测键盘是否有键按下
					CMFollowVal = followValCal(CMSwingVal);   //底盘跟随PID计算
					swing_dir		= 1;
					swing_count = 0;
					CMSwingVal 	= 0;
					move(keymove_x,keymove_y,CMFollowVal);	  //底盘运动函数
			 }
		 }
	}
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
		speedX *= gears_speedXYZ;
		speedY *= gears_speedXYZ;		
		rad *= gears_speedRAD;
	
		speedA = ( speedX + speedY + rad);
		speedB = ( speedX - speedY + rad);
		speedC = (-speedX - speedY + rad);
		speedD = (-speedX + speedY + rad);
	
	#ifdef Referee_System
		CMWatt_Cal();
		speedA = CMSpeedLegalize(speedA,speed_limite);
		speedB = CMSpeedLegalize(speedB,speed_limite);
		speedC = CMSpeedLegalize(speedC,speed_limite);
		speedD = CMSpeedLegalize(speedD,speed_limite);
	#else
	speedA = CMSpeedLegalize(speedA,7000);
	speedB = CMSpeedLegalize(speedB,7000);
	speedC = CMSpeedLegalize(speedC,7000);
	speedD = CMSpeedLegalize(speedD,7000);
	//无裁判系统时，速度上限函数		
	#endif
	CMControlOut(speedA,speedB,speedC,speedD);
}

/***************************************************************************************
 *Name        : keyboardmove
 *function    : 计算底盘电机速度给定值 由键盘控制
 *input       : keyboardvalue
 *output      : 无 
 *Description : 改变了全局变量	speedA, speedB, speedC, speedD

****************************新键盘*****************************************************/
void keyboardmove(uint16_t keyboardvalue)
{	
			xlimit = XLIMIT_OFFSET;
			ylimit = YLIMIT_OFFSET;
	
	if((keyboardvalue & KEY_PRESSED_OFFSET_Q )==KEY_PRESSED_OFFSET_Q )
	{
		quick_spin_flag=1;
		CMFollowVal_QUICK=-200;
	}
	else if((keyboardvalue & KEY_PRESSED_OFFSET_E )==KEY_PRESSED_OFFSET_E )
	{
		quick_spin_flag=1;
		CMFollowVal_QUICK=200;
	}
	else
	{
				quick_spin_flag=0;
		CMFollowVal_QUICK=0;
	}



	switch((keyboardvalue & (KEY_PRESSED_OFFSET_SHIFT)))
		{ 
			case (KEY_PRESSED_OFFSET_SHIFT):
				flag_swing = 1;
				break;
			default:
				flag_swing = 0;
				break;
	  }
		
		if(RC_Ex_Ctl.rc.s1 == 1) //Speed DOWN
		{
				xlimit = 100;
				ylimit = 80;
		}
		
	// W and S
	switch(keyboardvalue & (KEY_PRESSED_OFFSET_W|KEY_PRESSED_OFFSET_S))
		{   	
			case ( KEY_PRESSED_OFFSET_W):
				m = 0;
				if(keymove_y>0)    //正在向前
					keymove_y += 3*step;           //加步长
				else
					keymove_y += step*2;
				break;
			case ( KEY_PRESSED_OFFSET_S):
				m = 0;
				if(keymove_y<0)
					keymove_y -= 3*step;
				else 
					keymove_y -= step*2;
				break;
			default:      //没有输入时
				m++;
				if(m>1)
				{
					if(keymove_y>20){keymove_y=keymove_y-3*step;}
					else if(keymove_y<-20){keymove_y=keymove_y+3*step;}
					else{
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
					if(keymove_x>20){keymove_x=keymove_x-3*step;}
					else if(keymove_x<-20){keymove_x=keymove_x+3*step;}
					else{
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

/***************************************************************************************
 *Name     : CMControlOut
 *Function ：底盘电机速度输出，经过PID控制器，发送给电机驱动器，同时有各个轮子的速度限制
 *Input    ：speedA,speedB,speedC,speedD
 *Output   ：无 
 *Description : 底盘电机速度环和幅值函数
****************************************************************************************/
int16_t out_a=0;
void CMControlOut(int16_t speedA , int16_t speedB ,int16_t speedC ,int16_t speedD )
{
	
	
		speedA=PID_ControllerDriver(&SPCHIASSISA,speedA,current_cm_201);
		speedB=PID_ControllerDriver(&SPCHIASSISB,speedB,current_cm_202);
		speedC=PID_ControllerDriver(&SPCHIASSISC,speedC,current_cm_203);
		speedD=PID_ControllerDriver(&SPCHIASSISD,speedD,current_cm_204);
	/************    电机PID计算   *************/
//	speedA = PID_Driver(speedA, current_cm_201, &CMPid, 10000);    
//	speedB = PID_Driver(speedB, current_cm_202, &CMPid, 10000);    
//	speedC = PID_Driver(speedC, current_cm_203, &CMPid, 10000);    
//	speedD = PID_Driver(speedD, current_cm_204, &CMPid, 10000);    
	
	
	/************    功率限制    *************/
//	last_ele_dec = now_ele_dec;
//	now_ele_dec = ele_cal;
//	
//	if(now_ele_dec != last_ele_dec)
//	{
//		speedA = CMWatt_Cal(speedA,now_ele_dec);
//		speedB = CMWatt_Cal(speedB,now_ele_dec);
//		speedC = CMWatt_Cal(speedC,now_ele_dec);
//		speedD = CMWatt_Cal(speedD,now_ele_dec);
//	}
//	else
//	{
//	/************    幅值限制    *************/
//	#ifdef INFANTRY_3 
//	if(current_value/4096.0*3.3f < 2.5)
//		{
//			speedA = CMSpeedLegalize(speedA,2600);
//			speedB = CMSpeedLegalize(speedB,2600);
//			speedC = CMSpeedLegalize(speedC,2600);
//			speedD = CMSpeedLegalize(speedD,2600);
//		}
//	#else
		speedA = CMSpeedLegalize(speedA,3800);
		speedB = CMSpeedLegalize(speedB,3800);
		speedC = CMSpeedLegalize(speedC,3800);
		speedD = CMSpeedLegalize(speedD,3800);
//	#endif
		
//	}
out_a=speedA;   
	CAN1_Send_Bottom(speedA,speedB,speedC,speedD);
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

//	followVal = PID_Driver(Setposition, NowPosition, &CMFollowPid , 10000); 
	followVal=PID_ControllerDriver(&SPFOLLOW,Setposition,NowPosition);
	followVal = CMSpeedLegalize(followVal,1200);	
  //跟随量最小值，角度过小不跟随	
	if(abs(followVal) < followVal_limit) followVal = 0;
	
	return followVal;
}

/*-------------  底盘跟随角度控制  -------------*/
void SwingFollowValCal(void)
{
	if(flag_swing)
	{
//		if( swing_dir>0 )	
//		{
//			swing_count++;
//			CMSwingVal +=  40 ;
//		}
//		else
//		{		
//			CMSwingVal -=  40 ;
//		}
//		if(abs(CMSwingVal) >= 1200) {swing_dir *= -1;}
		swing_count++;
		CMSwingVal=1000*sin(2*PI*swing_count*0.005f)*cos(2*PI*swing_count*0.005f);
	}
	else
	{
		CMSwingVal = 0;
		swing_count=0;
	}

}

/*-------------  底盘电机速度PID和跟随PID初始化  -------------*/
void CMPID_Init(void)
{
	PID_Init(&CMPid, PIDVAL_CM_SPEED_p, 	PIDVAL_CM_SPEED_i, 	PIDVAL_CM_SPEED_d, -PIDVAL_CM_SPEED_limit,PIDVAL_CM_SPEED_limit,0);
	SetPIDCR(&CMPid,(1<<PEN)+(1<<IEN)+(1<<PIDEN)+(1<<INTERVAL)+(1<<DEN));
	PID_Init(&CMFollowPid, PIDVAL_CM_FOLLOW_p, 	PIDVAL_CM_FOLLOW_i, 	PIDVAL_CM_FOLLOW_d, -PIDVAL_CM_FOLLOW_limit,PIDVAL_CM_FOLLOW_limit,0);
	SetPIDCR(&CMFollowPid,(1<<PEN)+(1<<IEN)+(1<<PIDEN)+(1<<INTERVAL)+(1<<DEN));
}

float p=9;
float i=5.8;
float d=1;
/*底盘初始化*/
void CMControlInit(void)
{
	PID_ControllerInit(&SPFOLLOW,20,20,1100,0.01);
	PID_ControllerInit(&SPCHIASSISA,20,20,10000,0.01);
	PID_ControllerInit(&SPCHIASSISB,20,20,10000,0.01);
	PID_ControllerInit(&SPCHIASSISC,20,20,10000,0.01);
	PID_ControllerInit(&SPCHIASSISD,20,20,10000,0.01);
	
	PID_SetGains(&SPFOLLOW,0.35,0.01,1.5);
	PID_SetGains(&SPCHIASSISA,p,i,d);
	PID_SetGains(&SPCHIASSISB,p,i,d);
	PID_SetGains(&SPCHIASSISC,p,i,d);
	PID_SetGains(&SPCHIASSISD,p,i,d);
	
	
}

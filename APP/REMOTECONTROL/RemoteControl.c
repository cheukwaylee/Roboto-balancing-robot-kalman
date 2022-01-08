/************************************************************
 *File		:	RemoteControl.c
  *Author	:  @YangTianhao ,490999282@qq.com，@TangJiaxin ,tjx1024@126.com
 *Version	: V1.0
 *Update	: 2017.12.11
 *Description: 	Remote data analysis
                Change remoteState (state of the system);
 ************************************************************/

#include "main.h"

RemoteState_e remoteState = PREPARE_STATE;
RemoteState_e remoteState_ex = PREPARE_STATE;

unsigned char auto_aim_flag=0x00;//定义自瞄标志位

unsigned char big_power_flag=0x00;//定义大小幅标志位
int16_t auto_aim_count=0;        //定义自瞄延时检测
int16_t small_power_count=0;  //定义大小幅延时检测
int now_can_count=0;
int last_can_count=0;
u8 view_flag_spec=0;

int bigbuff_leftright = 0;

int firstINAIMING = 0;
int temp=0;
/*-------------  遥控器循环  -------------*/
void RemoteControlLoop(void)
{
    ItoE();												//更新Ex值，并用Ex值来进行操作
    RemoteStateFSM();             //判断此时遥控器状态
    GimbalSwitchJudge();         //判断遥控器模式切换
}

/*-------------  遥控器模式状态机  -------------*/
void RemoteStateFSM(void)
{
    if(flag_Ready)
    {
        switch(RC_Ex_Ctl.rc.s2)       //判断s2摇杆值，改变状态
        {
        case 0:
            remoteState = ERROR_STATE;						//出错模式
            break;
        case 1:
            remoteState = NORMAL_REMOTE_STATE;  	//遥控器控制模式
            break;
        case 2:
            remoteState = STANDBY_STATE;  				//待机模式
            break;
        case 3:
            remoteState = KEY_REMOTE_STATE;				//键鼠模式
            break;
        default:
            remoteState = PREPARE_STATE;          //初始化准备模式
            break;
        }
        if(remoteState == NORMAL_REMOTE_STATE)
        {

//				auto_aim_flag = 0;
//
//			if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_Q ) == KEY_PRESSED_OFFSET_Q)
//				{
//					 robotLevel = 1;
//				}
//
//			if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_W ) == KEY_PRESSED_OFFSET_W)
//				{
//					 robotLevel = 2;
//				}
//
//			if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_E ) == KEY_PRESSED_OFFSET_E)
//				{
//					 robotLevel = 3;
//				}
//
            firstIN = 1;
            direction_wait = 1;
        }
        if(remoteState == KEY_REMOTE_STATE)
        {
					  //sendtoComputer(0,1,0);
            //切换自瞄模式
            if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_C ) == KEY_PRESSED_OFFSET_C)//||(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_Z ) == KEY_PRESSED_OFFSET_Z 
            //if(1)
            {
                if(firstINAIMING == 0)
                {
                    YawTarget.Mechanical=continuous_current_position_205;
                    YawTarget.Gyroscope=Yaw*57.3f;
                    PitchTarget.Mechanical=continuous_current_position_206;
                    GimbalStop();
                    PIDOut_Whole_Yaw = 0;
                    PIDOut_Whole_Pit = 0;
                    auto_aim_yaw_temp = 0;
                    auto_aim_pit_temp = 0;
                    fram.yaw = 0;
                    fram.pitch = 0;

                    firstINAIMING = 1;
                }
                auto_aim_flag=0xFF;
            }
            else
            {
                auto_aim_flag=0x00;
                firstINAIMING = 0;
            }

						if(((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_Z ) == KEY_PRESSED_OFFSET_Z) || ((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_V ) == KEY_PRESSED_OFFSET_V)) //切换小符
						//if(1)
						{
							big_power_flag = 0xFF;
						}
						else
						{
							big_power_flag = 0x00;
						}

            if( big_power_flag == 0xFF||auto_aim_flag==0xFF)
            {
                remoteState=VIEW_STATE;
            }

        }
        else
        {
            big_power_flag = 0;
            firstIN = 1;
            direction_wait = 1;

            if_rotate_ok = 0;
            why_not_shoot = 0;
            small_power_delay = 0;
        }

        if(auto_aim_count>0)
            auto_aim_count--;
        if(small_power_count>0)
            small_power_count--;
    }
}

/***************************************************************************************
 *Name     : GimbalSwitchJudge
 *Function ：判断机器模式切换
 *Input    ：无
 *Output   ：无
 *Description : 判断机器是否是从待机模式或出错模式切换成正常模式，若是，则软起动
****************************************************************************************/
void GimbalSwitchJudge(void)
{
    if((remoteState_ex == STANDBY_STATE || remoteState_ex == ERROR_STATE) && remoteState == NORMAL_REMOTE_STATE )
    {
        remoteState = PREPARE_STATE;    //使机器进入初始化状态，云台处于初始化模式，软启动
        //flag_Ready = 0;                 //flag_Ready置零，保证切换至正常模式时云台软启动
        //GimbalPidInitPrepare();         //云台初始化pid，以较慢速度归中
        remoteState_ex = NORMAL_REMOTE_STATE;
    }
    if((remoteState_ex == STANDBY_STATE || remoteState_ex == ERROR_STATE) && remoteState == KEY_REMOTE_STATE )
    {
        remoteState = PREPARE_STATE;    //使机器进入初始化状态，云台处于初始化模式，软启动
        //flag_Ready = 0;                 //flag_Ready置零，保证切换至正常模式时云台软启动
        //GimbalPidInitPrepare();         //云台初始化pid，以较慢速度归中
        remoteState_ex = KEY_REMOTE_STATE;
    }

}

/*-------------  遥控器初始化  -------------*/
void RemoteControlInit(void)
{
    ItoE();
    remoteState = PREPARE_STATE;    //开机2s前不接收遥控器信号，这里程序自动赋值为PREPARE_STATE模式
}


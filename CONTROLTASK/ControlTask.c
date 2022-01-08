/************************************************************
 *File		:	main.c
 *Author	:  @YangTianhao ,490999282@qq.com，@TangJiaxin ,tjx1024@126.com
 *Version	: V1.0
 *Update	: 2017.12.11
 *Description: 	RM2018_INFANTRY project.
                Control the CM (Chassis motor) , Gimbal motor , Shoot motor BY Remote controller.
                USB CAN to connect motor made by DJI.
 ************************************************************/

#include "main.h"

uint32_t time_tick_1ms = 0; //1毫秒计数，以“分时”操作各子控制程序
RemoteState_e remoteState_pre=PREPARE_STATE;
uint16_t small_power_delay=0;//小符射击延时计数（shootcontrol.c里面用到）
int final_count=0;
int com_count=0;
void WholeInitTask(void)
{

    RemoteControlInit();
    ShootControlInit();
    auto_aim_init();
		auto_bigbuff_init();
#ifdef Gimbal_Move
    GimbalControlInit();
#endif

#ifdef _chassis_move
    CMControlInit();
#endif

    //提示音
    BUZZER_ON(1500);
    delay_ms(500);
    BUZZER_ON(2000);
    delay_ms(500);
    BUZZER_ON(2500);
    delay_ms(500);
    BUZZER_ON(3000);
    delay_ms(500);
    BUZZER_OFF();

}
/*-------------  主控制程序  -------------*/
uint8_t sre[] = "dsfs";
int graph_level=0;
int graph_color=5;
int graph_x=100;
int graph_y=150;
void Control_Task()
{
    time_tick_1ms++;
    RemoteControlLoop();
    //监控程序循环
    monitorControlLoop();
		send_com_control();
    /*if(if_rotate_ok == 1)//if_rotate_ok在自瞄程序里面有用，如果获取到视觉发来的数据就置1
    {
        small_power_delay++;//接收到大小符需要的云台位置信息到云台运动到指定位置的时间
    }*/
#ifdef Gimbal_Move
    GimbalControlLoop();
    if(time_tick_1ms%1000 == 0)
    {
        final_count=auto_aim_count_;
        com_count=upack_count;
        auto_aim_count_=0;
        upack_count=0;

    }
#endif

#ifdef AutoAim_USB
    USB_TaskLoop(time_tick_1ms);
#endif

    if(time_tick_1ms%3 == 0)
    {
		//sendtoComputer(0,1,0);
    }
		

    if(time_tick_1ms%2 == 0)
    {
        ShootControlLoop();
        update_from_dma();
    }

    if(time_tick_1ms%10 == 0)
    {

        //sendtoComputer(0,0,0);

#ifdef _chassis_move
        CMControlLoop();
#endif

    }

    if(time_tick_1ms%10 == 5)
    {

			#ifdef SUPER_CAPACITOR
        CapContorl();
			#endif
    }

    if(time_tick_1ms%1000 ==0 )
    {
        send_data_to_client();
    }
    if(time_tick_1ms%1000 == 500)
    {
			Rescue_Moni();
    }

			

    //出错模式，每秒闪8次
    if(time_tick_1ms%10 == 0)
    {
        //PowersendtoComputer();
        //GyrosendtoComputer();//由于数据量较大，开启会导致云台控制发散，开启时需禁云台
    }
    if(time_tick_1ms%125 == 0 && remoteState == ERROR_STATE)
    {
        LED_G_TOGGLE();
        LED_R_TOGGLE();
    }
    //待机模式，每秒闪2次
    if(time_tick_1ms%500 == 0 && remoteState == STANDBY_STATE)
    {
        LED_G_TOGGLE();
        LED_R_TOGGLE();
    }
    //正常模式，每秒闪1次
    if(time_tick_1ms%999 == 1 &&(remoteState == NORMAL_REMOTE_STATE||remoteState == KEY_REMOTE_STATE))
    {
        LED_G_TOGGLE();
        LED_R_TOGGLE();
    }
    if(time_tick_1ms == 10000)   //清零，防止计数过大
        time_tick_1ms = 0;
    remoteState_pre=remoteState;
}

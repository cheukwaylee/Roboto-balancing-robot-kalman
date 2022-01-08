#include "main.h"

frame frame_ex;
//frame fram;

/**************************************/

double  parameter_bigbuff_yaw=1;
double  parameter_bigbuff_pitch=1;
int 		get_target = 0;
int			be_static = 0;
int			send_finish = 0;
float		last_206 = 0;
float		last_205 = 0;
float		yaw_afterFilter = 0;
float		pitch_afterFilter = 0;
float		track_yaw_mean[21] = {0};
float		track_pitch_mean[21] = {0};

float		track_yaw_compare[21] = {0};
float		track_pitch_compare[21] = {0};
float		track_shoot_compare[21] = {0};
int			back2middle = 1;
uint16_t		shooter_heat0_last = 0;
int			firstIN = 1;
int			direction_wait = 2000;
int			last_shoot_seq = 0;
int			why_not_shoot = 0;
int			shoot_next = 0;
int			beforeShoot_yaw_pos = 0;
int			beforeShoot_pitch_pos = 0;
int			big_moveBegin = 0;
int			big_moveOver = 0;
/**************************************/
double  parameter_yaw=0.008;
double  parameter_pitch=0.005;
double  view_ch2;
double  view_ch3;
double  virual_ch2;
double  virual_ch3;

double  last_view_ch2;
double  last_view_ch3;
double  view_ch2_interval;
double  view_ch3_interval;

double  step_ch2=0.0;
double  step_ch3=0.0;
double  last_step_ch2=0.0;
double  last_step_ch3=0.0;

double  visual_yaw_kp=0.4;  //调试内容！！！
double  visual_yaw_kp_init=0.4;
double  visual_yaw_kp_temp=0.1;
double 	slow_point=0.00001;
float 	v_step=10;

double  visual_yaw_ki=0.3;
double  visual_yaw_kd=1.87; //0.45
double	integral_yaw=0.0;

double  visual_pitch_kp=1.0;
double  visual_pitch_ki=0.02;
double  visual_pitch_kd=0.8;

double  visual_yaw_kp_big=1;  //1.0
double  visual_yaw_kd_big=3;  //1.0
double  visual_pitch_kp_big=0.75;
double  visual_pitch_ki_big=0.02;
double  visual_pitch_kd_big=70;

double	integral_pitch=0.0;
double  now_yaw_angle=0.0;
double  last_yaw_angle=0.0;

double  visual_yaw_pid_out;
double  visual_pitch_pid_out;

int     FirstFlag=0;
int     time_count=0;
int     time_interval=40;
int     if_newframe = 0;
int     if_rotate_ok = 0;
int 		if_if_newframe=0;
int vie_temp=0;
/*************************************/
u8 auto_aim_renew_flag=0;
float auto_aim_yaw=0;
float auto_aim_pit=0;
float last_auto_aim_pit = 0;
float last_auto_aim_yaw = 0;

float auto_aim_yaw_temp=0;
float auto_aim_pit_temp=0;

float auto_aim_yaw_tar=0;
float auto_aim_pit_tar=0;

float bigbuff_yaw=0;
float bigbuff_pit=0;
float bigbuff_yaw_temp=0;
float bigbuff_pit_temp=0;

PID_Type auto_yaw,auto_pit;
PID_Type autobigbuff_yaw,autobigbuff_pit;

int timeout_count = 0;
float last_yaw = 0;
float last_pitch = 0;

int bigbuff_change = 0;
int if_newframe_bigbuff = 0;
int newframe_waiting = 0;

int reachPitch_Max = 0;
int reachPitch_Min = 0;
/************************************/

int upack_count=0;
uint32_t last_time_tick_1ms,last_last_time_tick_1ms;


float temp_yaw_target_p=0.0324f;
float temp_pitch_target_p = 0.0324f;


void auto_aim_init(void)
{

//	PID_ControllerInit(&auto_yaw,50,50,10,0.001);
//	PID_ControllerInit(&auto_pit,50,50,10,0.001);
    PID_ControllerInit(&auto_yaw,2,2,3,0.001);
    PID_ControllerInit(&auto_pit,2,2,3,0.001);

    PID_SetGains(&auto_yaw,0.0224,0,0);//0.000018);//0.0025,0.001,0 0.0025,0,0.000008
    PID_SetGains(&auto_pit,0.0086,0,0);//0.0003);//0.003,0.001,0 0.0035,0,0.0001
}


void auto_bigbuff_init(void)
{

//	PID_ControllerInit(&auto_yaw,50,50,10,0.001);
//	PID_ControllerInit(&auto_pit,50,50,10,0.001);
	
#ifdef INFANTRY_2
    PID_ControllerInit(&autobigbuff_yaw,2,2,3,0.001);
    PID_ControllerInit(&autobigbuff_pit,2,2,60,0.001);

    PID_SetGains(&autobigbuff_yaw,0.01,0.005,0);//0.0224,0,0 0.00000004
    PID_SetGains(&autobigbuff_pit,0.05,0,0.001);//0.003,0.001,0
#endif

#ifdef INFANTRY_1
		PID_ControllerInit(&autobigbuff_yaw,5,10,60,0.001);
    PID_ControllerInit(&autobigbuff_pit,5,2,60,0.001);

    PID_SetGains(&autobigbuff_yaw,0.2,0,0);//0.2,0,0small
    PID_SetGains(&autobigbuff_pit,0.015,0,0);//0.01,0,0
#endif
	
#ifdef INFANTRY_3
    PID_ControllerInit(&autobigbuff_yaw,5,2,60,0.001);
    PID_ControllerInit(&autobigbuff_pit,5,2,60,0.001);

    PID_SetGains(&autobigbuff_yaw,0.2,0,0);//0.0224,0,0 0.00000004
    PID_SetGains(&autobigbuff_pit,0.015,0,0);//0.003,0.001,0
#endif
}
int auto_aim_count_=0;
float yaw_bias=0.0f;
float pitch_bias=0.0f;
void Auto_aim(frame fram,int len)
{
    vie_temp=update_flag;//unpackFrame(rx_buf,len,&fram);
    if(vie_temp == 1)  //解包成功
    {
        auto_aim_count_++;
        if(fram.timestamp != frame_ex.timestamp)//如果前一帧数据和当前帧时间戳一样,目标丢失,不作为
        {

            upack_count++;
            /*if(shoot_next == 1 && newframe_waiting == 0)
            {
            	if_newframe_bigbuff = 0;
            	newframe_waiting = 1;
            }
            else if(shoot_next == 1 && newframe_waiting == 1)
            {
            	if_newframe_bigbuff = 1;
            }*/
            if_newframe = 1;
            auto_aim_renew_flag=1;

            timeout_count = 0;
        }
        else
        {
            if_newframe = 0;
            auto_aim_renew_flag=0;
					
						timeout_count++;
        }
        frame_ex.timestamp = fram.timestamp;
    }
    else
    {
        
    }
    if(timeout_count > 3000)
    {
        //fram.extra[0] = 0x33;
        YawTarget.Mechanical = MIDDLE_YAW;
        PitchTarget.Mechanical = MIDDLE_PITCH;
    }
    if(auto_aim_flag == 0xFF &&((fram.extra[0]==0x31) || (fram.extra[0]==0x32)))
    {
			yaw_bias-=(abs(RC_Ex_Ctl.mouse.x)>40 ? (RC_Ex_Ctl.mouse.x>0 ? 40: -40 ) : RC_Ex_Ctl.mouse.x) * MOUSE_YAW_SPEED*0.6f;
			pitch_bias+=5.0f* MOUSE_PIT_SPEED * RC_Ex_Ctl.mouse.y*0.6f;
			
			yaw_bias=CMSpeedLegalize(yaw_bias,30);
			pitch_bias=CMSpeedLegalize(pitch_bias,20);

        if(auto_aim_renew_flag==1)
        {
//				YawTarget.Gyroscope-=fram.yaw;
//				PitchTarget.Mechanical-=fram.pitch*22.75f;
//            auto_aim_yaw=YawTarget.Gyroscope-fram.yaw;
//            auto_aim_pit=PitchTarget.Mechanical-fram.pitch*22.75f;
						temp_yaw_target_p=0.0124f+0.015f*(1-fram.yaw/5.0f);
						if(temp_yaw_target_p<0.01f)
							temp_yaw_target_p=0.01f;
						PID_SetGains(&auto_yaw,temp_yaw_target_p,0,0);
            auto_aim_yaw=Yaw*57.3f-fram.yaw;
            auto_aim_pit=continuous_current_position_206-fram.pitch*22.75f;
						
						auto_aim_yaw+=yaw_bias;
						auto_aim_pit+=pitch_bias;

            if(auto_aim_pit > MIDDLE_PITCH+PitMax)
            {
                auto_aim_pit = MIDDLE_PITCH+PitMax;
                reachPitch_Max = 1;
            }
            else
            {
                reachPitch_Max = 0;
            }
            if(auto_aim_pit < MIDDLE_PITCH+PitMin)
            {
                auto_aim_pit = MIDDLE_PITCH+PitMin;
                reachPitch_Min = 1;
            }
            else
            {
                reachPitch_Min = 0;

            }
        }
        auto_aim_yaw_temp=PID_ControllerDriver(&auto_yaw,auto_aim_yaw,YawTarget.Gyroscope);
        auto_aim_pit_temp=PID_ControllerDriver(&auto_pit,auto_aim_pit,PitchTarget.Mechanical);
//
        YawTarget.Gyroscope+=auto_aim_yaw_temp;
        PitchTarget.Mechanical+=auto_aim_pit_temp;
//
        /*if(PitchTarget.Mechanical > MIDDLE_PITCH+PitMax)
        {
        	PitchTarget.Mechanical = MIDDLE_PITCH+PitMax;
        	reachPitch_Max = 1;
        }
        else
        {
        	reachPitch_Max = 0;
        }

        if(PitchTarget.Mechanical < MIDDLE_PITCH+PitMin)
        {
        	PitchTarget.Mechanical = MIDDLE_PITCH+PitMin;
        	reachPitch_Min = 1;
        }
        else
        {
        	reachPitch_Min = 0;
        }*/
    }
    else if(auto_aim_flag == 0xFF  &&((fram.extra[0]==0x33) || (fram.extra[0]==0x34)))
    {
        YawTarget.Gyroscope+=0;
        PitchTarget.Mechanical+=0;
    }
    else if(big_power_flag == 0xFF && auto_aim_flag == 0 && fram.extra[0]==0x34 && timeout_count < 3000)
    {
        if(auto_aim_renew_flag==1)
        {
#ifdef INFANTRY_2
						if(fabs(fram.yaw) > 1.0)
						{
							temp_yaw_target_p = 0.0124 + 0.015 - 0.1*fabs(fram.yaw)/5.0;
						}
						else
						{
							temp_yaw_target_p = 0.0124 + 0.015 + 0.04*fabs(fram.yaw)/5.0;
						}
						if(temp_yaw_target_p<0.01f)//0.01 0.002 0.003
							temp_yaw_target_p=0.01f;//0.01*/
						PID_SetGains(&autobigbuff_yaw,temp_yaw_target_p,0,0);
				
						PID_SetGains(&autobigbuff_pit,0.05,0,0.001);
						
            auto_aim_yaw=Yaw*57.3f-fram.yaw;
            auto_aim_pit=continuous_current_position_206-fram.pitch*22.75f;

            if(auto_aim_pit > MIDDLE_PITCH+PitMax)
            {
                auto_aim_pit = MIDDLE_PITCH+PitMax;
                reachPitch_Max = 1;
            }
            else
            {
                reachPitch_Max = 0;
            }
            if(auto_aim_pit < MIDDLE_PITCH+PitMin)
            {
                auto_aim_pit = MIDDLE_PITCH+PitMin;
                reachPitch_Min = 1;
            }
            else
            {
                reachPitch_Min = 0;

            }
#endif

#ifdef INFANTRY_1
						/*if(fabs(fram.yaw) > 1.5)
						{
							temp_yaw_target_p = 0.2;
						}
						else
						{
							temp_yaw_target_p = 0.2 + 0.4*fabs(fram.yaw);
						}
						//if(temp_yaw_target_p<0.2f)//0.01 0.002 0.003
							//temp_yaw_target_p=0.2f;//0.01
						PID_SetGains(&autobigbuff_yaw,temp_yaw_target_p,0,0.00005);*/
						
						auto_aim_yaw=continuous_current_position_205-fram.yaw*22.75f;
            auto_aim_pit=continuous_current_position_206-fram.pitch*22.75f;

            if(auto_aim_pit > MIDDLE_PITCH+PitMax)
            {
                auto_aim_pit = MIDDLE_PITCH+PitMax;
                reachPitch_Max = 1;
            }
            else
            {
                reachPitch_Max = 0;
            }
            if(auto_aim_pit < MIDDLE_PITCH+PitMin)
            {
                auto_aim_pit = MIDDLE_PITCH+PitMin;
                reachPitch_Min = 1;
            }
            else
            {
                reachPitch_Min = 0;

            }	
#endif

#ifdef INFANTRY_3
						auto_aim_yaw=continuous_current_position_205-fram.yaw*22.75f;
            auto_aim_pit=continuous_current_position_206-fram.pitch*22.75f;

            if(auto_aim_pit > MIDDLE_PITCH+PitMax)
            {
                auto_aim_pit = MIDDLE_PITCH+PitMax;
                reachPitch_Max = 1;
            }
            else
            {
                reachPitch_Max = 0;
            }
            if(auto_aim_pit < MIDDLE_PITCH+PitMin)
            {
                auto_aim_pit = MIDDLE_PITCH+PitMin;
                reachPitch_Min = 1;
            }
            else
            {
                reachPitch_Min = 0;

            }		
#endif
						
            
        }
				
				/*if(fabs(auto_aim_pit - last_auto_aim_pit) > 2.0*22.756 || fabs(auto_aim_yaw - last_auto_aim_yaw) > 4.0)
				{
					
				}
				else
				{*/
					//auto_aim_yaw_temp=PID_ControllerDriver(&autobigbuff_yaw,auto_aim_yaw,YawTarget.Gyroscope);
#ifdef INFANTRY_2
					auto_aim_yaw_temp=PID_ControllerDriver(&autobigbuff_yaw,auto_aim_yaw,YawTarget.Gyroscope);
#endif
				
#if ((defined INFANTRY_1) || (defined INFANTRY_3))
					auto_aim_yaw_temp=PID_ControllerDriver(&autobigbuff_yaw,auto_aim_yaw,YawTarget.Mechanical);
#endif
					auto_aim_pit_temp=PID_ControllerDriver(&autobigbuff_pit,auto_aim_pit,PitchTarget.Mechanical);
	//
#ifdef INFANTRY_2
					YawTarget.Gyroscope+=auto_aim_yaw_temp;
#endif
				
#if ((defined INFANTRY_1) || (defined INFANTRY_3))
					YawTarget.Mechanical+=auto_aim_yaw_temp;
#endif
					
					PitchTarget.Mechanical+=auto_aim_pit_temp;
				//}
				//last_auto_aim_pit = auto_aim_pit;
				//last_auto_aim_yaw = auto_aim_yaw;
        
				//char uart3_buf[24];
				//uint8_t size = sprintf(uart3_buf, "%f\n", auto_aim_pit_temp);
				//UART3_Send((uint8_t*)uart3_buf, size);
    }
}

bool glide_compare(float value_buf[],float ADNum, float limit_num)
{
    //float sum=0;
    int mi,count1,count2;
    for (mi = 0; mi < 5; mi ++)
    {
        value_buf[mi] = value_buf[mi+1];
    }
    value_buf[5] = ADNum;
    for(count1=0; count1<5; count1++)
    {
        for(count2=count1+1; count2<6-count1; count2++)
        {
            if(fabs(value_buf[count1] - value_buf[count2]) > limit_num)
                return false;
        }
    }
    return true;
}

/*float glide_filter(float value_buf[],float ADNum)
{
  float sum=0;
	int mi,count;
	for (mi = 0; mi < 5; mi ++)
	{
		value_buf[mi] = value_buf[mi+1];
	}
	value_buf[5] = ADNum;
  for(count=0;count<6;count++)
		sum+=value_buf[count];
  return (sum/6);
}*/

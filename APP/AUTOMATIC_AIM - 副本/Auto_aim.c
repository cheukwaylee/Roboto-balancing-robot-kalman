#include "main.h"

frame frame_ex;
frame fram;

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

double  visual_yaw_kp=3;  //1.0
double  visual_yaw_kp_init=3;  //1.0
double  visual_yaw_kp_temp=0.15;  //1.0
double 	slow_point=0.00001;
float 	v_step=10;

double  visual_yaw_ki=0.3;
double  visual_yaw_kd=1.87; //0.45
double	integral_yaw=0.0;

double  visual_pitch_kp=1.0;
double  visual_pitch_ki=0.02;
double  visual_pitch_kd=0.8;
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



uint32_t last_time_tick_1ms,last_last_time_tick_1ms;

void Auto_aim(u8 *rx_buf,int len)
{

	if(unpackFrame(rx_buf,len,&fram) == 0)  //解包成功
	{	

		if(fram.timestamp != frame_ex.timestamp)//如果前一帧数据和当前帧时间戳一样,目标丢失,不作为
			{
				
				if_newframe = 1;			
				
			}
			
				frame_ex.timestamp = fram.timestamp;
	 }
	if(auto_aim_flag == 0xFF && small_power_flag == 0x00)
		{
			if(if_newframe == 1)
		 {
			 	last_last_time_tick_1ms=last_time_tick_1ms;;
				last_time_tick_1ms=time_tick_1ms;
				if(FirstFlag==1)
				{
						last_view_ch2 = parameter_yaw*fram.yaw;
						last_view_ch3 = parameter_pitch*fram.pitch;
						view_ch2 = last_view_ch2;
						view_ch3 = last_view_ch3;
						view_ch2_interval=0;
						view_ch3_interval=0;
					  time_interval=30;
				}
				else
				{
						last_view_ch2 = view_ch2;
						last_view_ch3 = view_ch3;
					  view_ch2 = parameter_yaw*fram.yaw;
						view_ch3 = parameter_pitch*fram.pitch;
						view_ch2_interval=view_ch2-last_view_ch2;
						view_ch3_interval=view_ch3-last_view_ch3;
					  time_interval=last_time_tick_1ms-last_last_time_tick_1ms;
					  if(time_interval<0)
							time_interval=time_interval+10000;
            if(FirstFlag<=100)
							FirstFlag++;
				}

					  time_count=0;
			}
		 
			 if(time_count<60)
		{
			last_step_ch2=step_ch2;
			last_step_ch3=step_ch3;
			
			step_ch2=	view_ch2; //+view_ch2_interval/time_interval*time_count;//线性插值预测
			step_ch3=	view_ch3 +view_ch3_interval/time_interval*time_count;//线性插值预测
			
			if(abs(step_ch2)<slow_point)
			{
				visual_yaw_kp=visual_yaw_kp_temp*(1-abs(step_ch2)/slow_point);
			}
			else
			{
				visual_yaw_kp=visual_yaw_kp_init;
			}
			
			visual_yaw_pid_out=visual_yaw_kp*step_ch2+visual_yaw_kd*(step_ch2-last_step_ch2);//PD控制
			visual_pitch_pid_out=visual_pitch_kp*step_ch3+visual_pitch_kd*(step_ch3-last_step_ch3);//PD控制
			
		#ifdef gongkong
			if(FirstFlag<=20)
			{
			  YawTarget.Gyroscope    += fabs(visual_yaw_pid_out)< v_step ? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? v_step:-v_step);
			  PitchTarget.Mechanical  += (fabs(visual_pitch_pid_out)<1.75? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 1.75 : -1.75))*22.75f;
			}
			else
			{
				YawTarget.Gyroscope    += fabs(visual_yaw_pid_out)< 1.0? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
			  PitchTarget.Mechanical  += (fabs(visual_pitch_pid_out)<3.0? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 3.0 : -3.0))*22.75f;
			}
		#endif
				

		}
			time_count++;
		
		 
		if(time_count>100)
		{
			 FirstFlag=1;
		}
		
			 if_newframe = 0;
		 
		}
	 else if(big_power_flag == 0xFF && auto_aim_flag == 0)
	 { 
		 if(if_newframe == 1)
		 {
			 YawTarget.Mechanical    += fabs(fram.yaw)< 16.0?  fram.yaw :( fram.yaw>0 ? 16.0:-16.0);
	     PitchTarget.Mechanical  += (fabs(fram.pitch)<7.0? fram.pitch :(fram.pitch>0 ? 7.0 : -7.0))*22.75f;
			 
			 if_newframe = 0;
			
			 if_rotate_ok = 1;
		 }	 
	 }
	 
	 

 
	
	 
}

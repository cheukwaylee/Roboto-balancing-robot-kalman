#include "main.h"

u8 cap_boost_flag=0;
u8 cap_buck_flag=0;
u8 full_buck_flag=0;

void CapContorl(void)
{
	if(cap_receive.CapVol<11800)
	{
		cap_boost_flag=0;
	}
	else if(cap_receive.CapVol>13000)
	{
		cap_boost_flag=1;
	}
	
	if(cap_receive.CapVol>15400)
	{
		cap_buck_flag=0;
	}
	else if(cap_receive.CapVol<15000)
 	{
		cap_buck_flag=1;
	}
	
	if(remoteState == STANDBY_STATE)
	{
		if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_C )==KEY_PRESSED_OFFSET_C)
		{
			full_buck_flag=1;
		}
	}
	else
	{
		full_buck_flag=0;
	}
	
	
	
	if(remoteState == KEY_REMOTE_STATE||remoteState == NORMAL_REMOTE_STATE||remoteState == VIEW_STATE)
	{
		
		if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_F )==KEY_PRESSED_OFFSET_F&&cap_boost_flag==1 )
		{
			Cap_Update_Cmd(Cap_Run,Cap_Boost,Cap_change,40000);
		}
		else if(cap_buck_flag==1)
		{
			
			if(output_current_sum<2500)
			{
				u16 buck_power=(2500-output_current_sum)*19.53125f;//   (x/16384.0f*20*24)*1000/1.5
				Cap_Update_Cmd(Cap_Run,Cap_Buck,Cap_change,buck_power);
			}
			else
			{
				Cap_Update_Cmd(Cap_Run,Cap_Buck,Cap_change,0);
			}

		}
		else
		{
			Cap_Update_Cmd(Cap_Run,Cap_Buck,Cap_change,0);
		}
	}
	else if(remoteState == STANDBY_STATE&&full_buck_flag==1&&cap_buck_flag==1)
	{
		Cap_Update_Cmd(Cap_Run,Cap_Buck,Cap_change,50000);//75W³äµç
	}
	else
	{
		Cap_Update_Cmd(Cap_Run,Cap_Buck,Cap_change,0);
	}
	
}

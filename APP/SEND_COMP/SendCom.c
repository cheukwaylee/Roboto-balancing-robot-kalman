#include "main.h"

int send_time_counter=0;
RemoteState_e pre_remoteState = KEY_REMOTE_STATE;
int entering_auto_aim=0;
void send_com_control(void)
{
	send_time_counter++;
	
	
	 if(remoteState == VIEW_STATE )
	 {
		 if(pre_remoteState!=remoteState)
		 {
			 entering_auto_aim=1;
		 }
//		 else
//		 {
//			 entering_auto_aim=0;
//		 }
		 
		 
		 
		 if(auto_aim_flag == 0xFF && big_power_flag == 0x00)
		 {
			 if(send_time_counter%5==0)
			 {
				 sendtoComputer(0,1,0,entering_auto_aim);
				 if(entering_auto_aim)
					 entering_auto_aim=0;
			 }
		 }
		 else if(big_power_flag == 0xFF && auto_aim_flag == 0x00)
		 {
			 if(send_time_counter%5==0)
			 {
				 if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_Z ) == KEY_PRESSED_OFFSET_Z&&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_V ) != KEY_PRESSED_OFFSET_V)
					{
						sendtoComputer(0,0,1,0);
					}
					else if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_V ) == KEY_PRESSED_OFFSET_V&&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_Z ) != KEY_PRESSED_OFFSET_Z)
					{
						sendtoComputer(0,0,2,0);
					}
					else if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_V ) == KEY_PRESSED_OFFSET_V&&(RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_Z ) == KEY_PRESSED_OFFSET_Z)
					{
						sendtoComputer(0,0,3,0);
					}
				}
		 }
	 }
	 
	 
	 pre_remoteState=remoteState;
	
	
	if(send_time_counter==9999)
	{
		send_time_counter=0;
	}
}


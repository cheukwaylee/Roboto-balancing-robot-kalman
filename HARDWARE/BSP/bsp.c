#include "main.h"
LPF_FirstOrder_type filter_206;
LPF_FirstOrder_type filter_205;
LPF_FirstOrder_type filter_207;

void BSP_Init(void)	//注意初始化的顺序
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(180);
	Buzzer_Init();
	Led_Configuration();
	Power_Configuration();
	MiniPC_Configuration();
	LPF_FirstOrder_Init(&filter_206,10,1000);//10 1000
	LPF_FirstOrder_Init(&filter_205,30,1000);	
	LPF_FirstOrder_Init(&filter_207,30,1000);
	
	SPI4Init();
	SPI5Init();
	ADIS16470_Init();
	MPU6500_Init();
	delay_ms(3000);
	init_euler();
	#ifdef AutoAim_USB
		USB_TaskInit();
	#endif
	USART6_Configuration(115200); 
	USART6_DMA_Init();
	USART2_DMA_Init();
	USART3_DMA_Init();
	
	
	TIM9_Init();
	TIM2_Init();
	RC_Init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_6tq,5,CAN_Mode_Normal);	
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_6tq,5,CAN_Mode_Normal);
	TIM5_Init();
	TIM4_Init();
	WholeInitTask();	
	delay_ms(1000);
	TIM6_Init();

	Laser_Configuration();
	
	
}





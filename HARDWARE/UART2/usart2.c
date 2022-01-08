
#include "main.h"
#include "crc_table.h"

//char Auto_Data_Raw[15];
frame Auto_Frame_Large_Raw;
frame fram;
u8 update_flag=0;;
//frame_update Auto_Frame_Large;
uint16_t Auto_Error_Count = 0;

frame sendtoCom_frame;
u8 sendbuffer[15];

void USART2_DMA_Init()
{
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);		
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6, GPIO_AF_USART2);
	
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart2;
	gpio.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &gpio);

	USART_DeInit(USART2);
	usart2.USART_BaudRate = 115200;
	usart2.USART_WordLength = USART_WordLength_8b;
	usart2.USART_StopBits = USART_StopBits_1;
	usart2.USART_Parity = USART_Parity_No;
	usart2.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2,&usart2);
	
//	USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
//	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); 
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	
	USART_Cmd(USART2,ENABLE);
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	{
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 4;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	}
	
	
	
	DMA_InitTypeDef dma,dma2;
	DMA_DeInit(DMA1_Stream5);
	DMA_DeInit(DMA1_Stream6); 
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	dma.DMA_Memory0BaseAddr = (uint32_t)(/*Auto_Data_Raw*/&Auto_Frame_Large_Raw.head);
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = AUTO_FRAME_LENGTH;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_Mode_Normal;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5,&dma);
	
	dma2.DMA_Channel = DMA_Channel_4;
	dma2.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	dma2.DMA_Memory0BaseAddr = (uint32_t)sendbuffer;   
	dma2.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma2.DMA_BufferSize = 15;       
	dma2.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma2.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma2.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma2.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma2.DMA_Mode = DMA_Mode_Normal ;
	dma2.DMA_Priority = DMA_Priority_High;
	dma2.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma2.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	dma2.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma2.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6,&dma2);
	{
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = DMA1_Stream6_IRQn;      
	nvic.NVIC_IRQChannelPreemptionPriority = 3;
	nvic.NVIC_IRQChannelSubPriority = 3;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);  
	}
	
	DMA_Cmd(DMA1_Stream5,DISABLE);							
	DMA_Cmd(DMA1_Stream5,ENABLE);
	
}

u8 Auto_Frame_Large_Unpack()
{
	if(Auto_Frame_Large_Raw.head != AUTO_FRAME_HEAD)
	{
		return 0;
	}
	u8 crc_check_auto = crc8Check((u8 *)&Auto_Frame_Large_Raw,AUTO_FRAME_LENGTH-2);
	if(Auto_Frame_Large_Raw.crc8check != crc_check_auto)
	{
		return 0;
	}
	memcpy((u8 *)&fram,(u8 *)&Auto_Frame_Large_Raw,AUTO_FRAME_LENGTH);
//	Auto_Frame_Large.pitch = Auto_Frame_Large_Raw.pitch;
//	Auto_Frame_Large.yaw = -Auto_Frame_Large_Raw.yaw;
//	Auto_Frame_Large.pitch_current = Gimbal_Standard(CAN1_206.Current_Position, Middle_Pitch_Large)/8192.0*360;
//	Auto_Frame_Large.yaw_current = Gimbal_Standard_Gyro(Gyro_Result.Yaw*180/3.14 - Large_Offset_Gyro.Yaw*180/3.14);
	
	Auto_Frame_Large_Raw.head = 0;
	Auto_Frame_Large_Raw.crc8check = 0;
	return 1;
}

uint16_t Auto_len = 0;
//float time_check = 0;
//u16 time_count = 0;
//u16 time_reset = 0;
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
//		static 
		Auto_len = USART2->DR;
		Auto_len = DMA_GetCurrDataCounter(DMA1_Stream5); 
		DMA_Cmd(DMA1_Stream5, DISABLE);
		
//		if(time_reset == 0)
//		{
//			time_reset = Auto_Frame.timestamp + 100;
//			time_count = 0;
//		}
//		else
//		{
//			
//			if(Auto_Frame.timestamp>=time_reset)
//			{
//				time_check = time_count*1.0/100;
//				time_reset = 0;
//			}
//			else
//			{
//				time_count ++;
//			}
//		}
		
		
//		if(Auto_len == 10)
//		{
//			
//			Auto_Error_Count = 0;
//		}
//		else
//			Auto_Error_Count ++;
		update_flag= Auto_Frame_Large_Unpack();
		DMA_SetCurrDataCounter(DMA1_Stream5,AUTO_FRAME_LENGTH);  //设置传输数据长度
    DMA_Cmd(DMA1_Stream5,ENABLE);  //打开DMA
		
	}
}


u8 crc8Check(u8 *buff,u16 len)
{
	u8 ucIndex,ucCRC8=(u8)CRC8_INIT;
	while (len--)
	{
		ucIndex = ucCRC8^(*buff++);
		ucCRC8 = CRC8_Table[ucIndex];
	}
	return(ucCRC8);
}

void packFrame(u8 *buff,frame *fram)
{	
	memcpy(buff,fram,sizeof(frame));
	buff[sizeof(frame)-2] = crc8Check((u8 *)fram,sizeof(frame)-2);
}

void DMA1_Stream6_IRQHandler(void)
{
//		if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF4))
//		{
//			DMA_ClearFlag(DMA1_Stream6, DMA_IT_TCIF4);
//			
//			//USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);			
////			DMA_Cmd(DMA1_Stream6,DISABLE);
//		}
	 if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//??DMA2_Steam7????  
    {   
        DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//??DMA2_Steam7??????  
    }  
}


void sendtoComputer(int timestamp_doing, int auto_aim, int big_buff,int entering_auto_aim)
{
//	sendtoCom_frame.yaw = (current_position_205-MIDDLE_YAW)*0.0439;
//	sendtoCom_frame.pitch =(current_position_206-MIDDLE_PITCH)*0.0439;
	sendtoCom_frame.head=0xf1;
	sendtoCom_frame.end=0xf2;
	
	sendtoCom_frame.timestamp = timestamp_doing;
	sendtoCom_frame.yaw = Yaw;
	sendtoCom_frame.pitch = Pitch;
	if(auto_aim == 1)
		sendtoCom_frame.extra[0] = 0x11;
	if(big_buff == 1)
		sendtoCom_frame.extra[0] = 0x21;// Smallbuff
	if(big_buff == 2)
		sendtoCom_frame.extra[0] = 0x22;// Clockwise
	if(big_buff == 3)
		sendtoCom_frame.extra[0] = 0x23;// Counterclockwise

	if(entering_auto_aim==1)
		sendtoCom_frame.extra[1] = 0x01;
	else
		sendtoCom_frame.extra[1] = 0x00;
	
	packFrame(sendbuffer,&sendtoCom_frame);//减少每次搬运内存时间
//  DMA_Cmd(DMA1_Stream6,ENABLE);
//	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	//DMA_Cmd(DMA1_Stream6,DISABLE);
	
	
	
	DMA_Cmd(DMA1_Stream6, DISABLE);                      //??DMA??   
      
    while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}  //??DMA?????    
          
    DMA_SetCurrDataCounter(DMA1_Stream6,15);          //?????    
		
    DMA_Cmd(DMA1_Stream6, ENABLE);     
}

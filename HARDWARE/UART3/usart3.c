/************************************************************
 *File		:	usart3.c
 *Author	:  @YangTianhao ,490999282@qq.com，@TangJiaxin ,tjx1024@126.com	@Jason , tjx1024@126.com
 *Version	: V1.0
 *Update	: 2017.12.11
 *Description: 	Usart3, for view data communication
                PD8 -> USART3_TX
								PD9 -> USART3_RX
 ************************************************************/
#include "usart3.h"
#include "stm32f4xx.h"

u8  USART_RX_BUF3[MAXLBUF_3];  
u8 sendbuffer3[100];

void USART3_DMA_Init(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8, GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9, GPIO_AF_USART3);
	/* -------------- Configure GPIO ---------------------------------------*/
		{
				GPIO_InitTypeDef gpio;
				USART_InitTypeDef usart3;
				gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
				gpio.GPIO_Mode = GPIO_Mode_AF;
				gpio.GPIO_OType = GPIO_OType_PP;
				gpio.GPIO_Speed = GPIO_Speed_100MHz;
				gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
				GPIO_Init(GPIOD, &gpio);
			
				USART_DeInit(USART3);
				usart3.USART_BaudRate = 115200;
				usart3.USART_WordLength = USART_WordLength_8b;
				usart3.USART_StopBits = USART_StopBits_1;
				usart3.USART_Parity = USART_Parity_No ;   
				usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx; 
				usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
				USART_Init(USART3,&usart3);
				USART_Cmd(USART3,ENABLE);
				USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
			  USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
		}		

		/* -------------- Configure NVIC ---------------------------------------*/
		{
				NVIC_InitTypeDef nvic;
				nvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;      
				nvic.NVIC_IRQChannelPreemptionPriority = 2;
				nvic.NVIC_IRQChannelSubPriority = 3;
				nvic.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&nvic);
		}
		
				/* -------------- Configure NVIC ---------------------------------------*/
		{
				NVIC_InitTypeDef nvic;
				nvic.NVIC_IRQChannel = DMA1_Stream4_IRQn;      
				nvic.NVIC_IRQChannelPreemptionPriority = 3;
				nvic.NVIC_IRQChannelSubPriority = 3;
				nvic.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&nvic);
		}
		
		/* -------------- Configure DMA -----------------------------------------*/
		{
				DMA_InitTypeDef dma1,dma2;
				DMA_DeInit(DMA1_Stream1);
			  DMA_DeInit(DMA1_Stream4);
			
				dma1.DMA_Channel = DMA_Channel_4;
				dma1.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
				dma1.DMA_Memory0BaseAddr = (uint32_t)USART_RX_BUF3;   
				dma1.DMA_DIR = DMA_DIR_PeripheralToMemory;
				dma1.DMA_BufferSize = MAXLBUF_3;       
				dma1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
				dma1.DMA_MemoryInc = DMA_MemoryInc_Enable;
				dma1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
				dma1.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
				dma1.DMA_Mode = DMA_Mode_Circular;
				dma1.DMA_Priority = DMA_Priority_VeryHigh;
				dma1.DMA_FIFOMode = DMA_FIFOMode_Disable;
				dma1.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
				dma1.DMA_MemoryBurst = DMA_Mode_Normal;
				dma1.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
				DMA_Init(DMA1_Stream1,&dma1);
			  
				dma2.DMA_Channel = DMA_Channel_7;
				dma2.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
				dma2.DMA_Memory0BaseAddr = (uint32_t)sendbuffer3;   
				dma2.DMA_DIR = DMA_DIR_MemoryToPeripheral;
				dma2.DMA_BufferSize = 23;       
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
				DMA_Init(DMA1_Stream4,&dma2);
			
			
				DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
				DMA_Cmd(DMA1_Stream1,ENABLE);

				DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);
				DMA_Cmd(DMA1_Stream4,ENABLE);


		}	

}

void DMA1_Stream1_IRQHandler(void)
{
		if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
		{
			DMA_ClearFlag(DMA1_Stream1, DMA_IT_TCIF1);
			DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);	
		}
}

void DMA1_Stream4_IRQHandler(void)
{
		if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))
		{
			DMA_ClearFlag(DMA1_Stream4, DMA_IT_TCIF4);		
		}
}

void USART3_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		USART_ITConfig(USART3,USART_IT_TC,DISABLE);
	}
}
void UART3_Send(u8 *buffer, u8 len)
{
	u8 i=0;
	while (i < len) 
	{ 
		while((USART3->SR&0X40)==0); //循环发送,直到发送完毕   
		USART3->DR = (u8) buffer[i++];      
	} 
}


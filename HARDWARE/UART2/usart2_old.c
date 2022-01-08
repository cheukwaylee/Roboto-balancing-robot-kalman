/************************************************************
 *File		:	usart3.c
 *Author	:  @YangTianhao ,490999282@qq.com，@TangJiaxin ,tjx1024@126.com	@Jason , tjx1024@126.com
 *Version	: V1.0
 *Update	: 2017.12.11
 *Description: 	Usart3, for view data communication
                PD8 -> USART3_TX
								PD9 -> USART3_RX
 ************************************************************/
#include "usart2.h"
#include "stm32f4xx.h"

u8  USART_RX_BUF2[MAXLBUF];  


//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}
#endif

void USART2_DMA_Init(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5, GPIO_AF_USART2); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6, GPIO_AF_USART2);
	/* -------------- Configure GPIO ---------------------------------------*/
		{
				GPIO_InitTypeDef gpio;
				USART_InitTypeDef usart2;
				gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
				gpio.GPIO_Mode = GPIO_Mode_AF;
				gpio.GPIO_OType = GPIO_OType_PP;
				gpio.GPIO_Speed = GPIO_Speed_100MHz;
				gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
				GPIO_Init(GPIOD, &gpio);
			
				USART_DeInit(USART2);
				usart2.USART_BaudRate = 115200;
				usart2.USART_WordLength = USART_WordLength_8b;
				usart2.USART_StopBits = USART_StopBits_1;
				usart2.USART_Parity = USART_Parity_No ;   
				usart2.USART_Mode = USART_Mode_Tx|USART_Mode_Rx; 
				usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
				USART_Init(USART2,&usart2);
				USART_Cmd(USART2,ENABLE);
				USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
			  USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
		}		

		/* -------------- Configure NVIC ---------------------------------------*/
		{
				NVIC_InitTypeDef nvic;
				nvic.NVIC_IRQChannel = DMA1_Stream5_IRQn;      
				nvic.NVIC_IRQChannelPreemptionPriority = 2;
				nvic.NVIC_IRQChannelSubPriority = 3;
				nvic.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&nvic);
		}
		
				/* -------------- Configure NVIC ---------------------------------------*/
		{
				NVIC_InitTypeDef nvic;
				nvic.NVIC_IRQChannel = DMA1_Stream6_IRQn;      
				nvic.NVIC_IRQChannelPreemptionPriority = 3;
				nvic.NVIC_IRQChannelSubPriority = 3;
				nvic.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&nvic);
		}
		
		/* -------------- Configure DMA -----------------------------------------*/
		{
				DMA_InitTypeDef dma1,dma2;
				DMA_DeInit(DMA1_Stream5);
			  DMA_DeInit(DMA1_Stream6);
			
				dma1.DMA_Channel = DMA_Channel_4;
				dma1.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
				dma1.DMA_Memory0BaseAddr = (uint32_t)USART_RX_BUF2;   
				dma1.DMA_DIR = DMA_DIR_PeripheralToMemory;
				dma1.DMA_BufferSize = MAXLBUF;       
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
				DMA_Init(DMA1_Stream5,&dma1);
			  
				dma2.DMA_Channel = DMA_Channel_4;
				dma2.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
				dma2.DMA_Memory0BaseAddr = (uint32_t)sendbuffer;   
				dma2.DMA_DIR = DMA_DIR_MemoryToPeripheral;
				dma2.DMA_BufferSize = 23;       
				dma2.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
				dma2.DMA_MemoryInc = DMA_MemoryInc_Enable;
				dma2.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
				dma2.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
				dma2.DMA_Mode = DMA_Mode_Circular ;
				dma2.DMA_Priority = DMA_Priority_High;
				dma2.DMA_FIFOMode = DMA_FIFOMode_Disable;
				dma2.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
				dma2.DMA_MemoryBurst = DMA_MemoryBurst_Single;
				dma2.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
				DMA_Init(DMA1_Stream6,&dma2);
			
			
				DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
				DMA_Cmd(DMA1_Stream5,ENABLE);

				//DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);


		}	
		//sendtoComputerInit();
}

void DMA1_Stream5_IRQHandler(void)
{
		if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF1))
		{
			DMA_ClearFlag(DMA1_Stream5, DMA_IT_TCIF1);
			DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF1);	
		}
}

void DMA1_Stream6_IRQHandler(void)
{
		if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF4))
		{
			DMA_ClearFlag(DMA1_Stream6, DMA_IT_TCIF4);			
		}
}

void USART2_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		USART_ITConfig(USART2,USART_IT_TC,DISABLE); 
	}
}
void USART2_Send(u8 *buffer, u8 len)
{
	u8 i=0;
	while (i < len) 
	{ 
		while((USART2->SR&0X40)==0); //循环发送,直到发送完毕   
		USART2->DR = (u8) buffer[i++];      
	} 
}

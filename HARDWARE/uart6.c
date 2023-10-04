#include "task.h"
#include "delay.h"
#include "uart6.h"
#include "gpio.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "SCServo.h" // 飞特串口舵机接口

//////////////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif


//UART 读数据缓冲区
__IO uint8_t uart6Buf[128];
__IO int head = 0;
__IO int tail  = 0;

static uint8_t rx_buffer[256]; // 接收缓冲区
static uint16_t rx_index = 0;  // 接收缓冲区索引
static uint8_t header_count = 0; // 用于检测0xFFFF开头

// 有关STS3032的反馈参数
uint16_t STS3032_Pos;
int STS3032_Speed;
int STS3032_Load;
int STS3032_Voltage;
int STS3032_Temper;
int STS3032_Move;
int STS3032_Current;

void Uart6_Flush(void)
{
	head = tail = 0;
}

int16_t Uart6_Read(void)
{
	if(head!=tail){
		uint8_t Data = uart6Buf[head];
		head =  (head+1)%128;
		return Data;
	}else{
		return -1;
	}
}

void USART6_Init(void) 
{                
		USART_InitTypeDef usart;
    GPIO_InitTypeDef  gpio;
	  NVIC_InitTypeDef  nvic;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
 
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); 
		GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); 
	
		gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; 
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOG,&gpio); 

		usart.USART_BaudRate = 1000000;//波特率设置
		usart.USART_WordLength = USART_WordLength_8b;
		usart.USART_StopBits = USART_StopBits_1;
		usart.USART_Parity = USART_Parity_No;
		usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;	
		USART_Init(USART6, &usart); 
		
		nvic.NVIC_IRQChannel = USART6_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority=2;
		nvic.NVIC_IRQChannelSubPriority =2;		
		nvic.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&nvic);	 
		
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART6,ENABLE);
}


//u8 Receive_6;

void USART6_IRQHandler(void)
{  				
		if(USART_GetFlagStatus(USART6,USART_FLAG_ORE)!=RESET)
		{
		(void)USART6->SR;   
		(void)USART6->DR;
		return;
		}
		if (USART_GetFlagStatus(USART6, USART_FLAG_ORE) != RESET)//防止接受数据太快导致溢出
		{  	
		//清除中断标志    		 
		(void)USART6->SR;   
		(void)USART6->DR;
		return;			
		}
    if (USART_GetITStatus(USART6, USART_IT_RXNE)) {
        uint8_t byte = USART_ReceiveData(USART6); // 读取接收到的字节

        // 检查数据包开头0xFFFF
        if (byte == 0xFF && header_count == 0) {
            header_count++;
        } else if (byte == 0xFF && header_count == 1) {
            header_count = 0;
            rx_index = 0;
            rx_buffer[rx_index++] = 0xFF;
            rx_buffer[rx_index++] = 0xFF;
        } else if (rx_index > 0) {
            rx_buffer[rx_index++] = byte;

            // 当接收到预期数量的字节后
            if (rx_index == 8) { // sts3032舵机的回传信息
                // 计算接收到的数据的checksum校验码
                uint8_t calculated_checksum = ~(rx_buffer[2]+rx_buffer[3]+rx_buffer[4]+rx_buffer[5]+rx_buffer[6]); // 不包括checksum和帧头字段

                // 提取接收到的checksum校验码
                uint8_t received_checksum = rx_buffer[rx_index - 1];

                // 对比校验码
                if (calculated_checksum == received_checksum) {
										STS3032_Pos =  (rx_buffer[5] << 8) | rx_buffer[6];
										
                } else {
                    // 校验失败，可能需要错误处理
                }

                // 重置接收缓冲区索引
                rx_index = 0;
            }
        }
    }

    // 清除接收中断标志
		USART_ClearFlag(USART6, USART_FLAG_RXNE);  
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);   
//		OSIntExit(); 
}

//void USART6_Handler(void)
//{
//	
//	  USART_ClearFlag(USART6,USART_FLAG_TC);
//		USART_SendData(USART6,0x64);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);//判断是否发送完成

//}

void Uart6_Send(uint8_t *buf , uint8_t len)
{
	uint8_t i=0;
	for(i=0; i<len; i++){
		USART_SendData(USART6, buf[i]);
		while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
	}
	while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
}










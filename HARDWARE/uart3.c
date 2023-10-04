// 引入相关头文件
#include "can1.h"
#include "task.h"
#include "delay.h"
#include "uart1.h"
#include "uart3.h"
#include "uart6.h"
#include "pwm.h"
#include "imu.h"
#include "gpio.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"

// 如果使用ucos操作系统，需要包含这个头文件
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif

// CRC-16 Modbus查表法
 const uint16_t crc16_table[256] = {
0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

// 计算CRC-16 Modbus校验码
uint16_t calc_crc16_modbus(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF; // 初始值
    for (uint16_t i = 0; i < length; ++i) {
				uint8_t pos = crc ^ data[i];
        crc = crc16_table[pos] ^ (crc >> 8);
    }
    return crc;
}

// 重定义fputc函数，用于USART3的数据发送
int fputc(int ch,FILE *f)
{
	USART_SendData(USART3,ch); // 发送数据
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	return ch; 
	
}



// USART3初始化函数
void TX2_USART3_Init(void)
{                
		USART_InitTypeDef usart;	// USART初始化结构体
    GPIO_InitTypeDef  gpio;		// GPIO初始化结构体
	  NVIC_InitTypeDef  nvic;		// NVIC初始化结构体
	
		// 使能相关时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		// 配置GPIO的复用功能
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); 
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); 
		// GPIO初始化设置
		gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; 
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOD,&gpio); 
		// USART初始化设置
		usart.USART_BaudRate = 115200;//波特率设置
		usart.USART_WordLength = USART_WordLength_8b;
		usart.USART_StopBits = USART_StopBits_1;
		usart.USART_Parity = USART_Parity_No;
		usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		usart.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;	
		USART_Init(USART3, &usart); 
		// NVIC初始化设置
		nvic.NVIC_IRQChannel = USART3_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority=1;
		nvic.NVIC_IRQChannelSubPriority =1;		
		nvic.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&nvic);	 
		// 使能USART3的接收中断
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		// 使能USART3
		USART_Cmd(USART3,ENABLE);
}


static uint8_t rx_buffer[256]; // 接收缓冲区
static uint16_t rx_index = 0;  // 接收缓冲区索引
static uint16_t expected_length = 0; // 预期的数据长度
static uint8_t header_count = 0; // 用于检测0xAA55开头

// 定义一些全局变量
u8 Receive;
u8 NewData_flag=0;
uint8_t controller_address; // 控制器地址
uint8_t function_code;      // 功能码

// 初始化电机信息数组，每个绳驱电机有一个地址和一个速度
// 假设最多有12个电机（单臂蛇形连续体）
int32_t snake_motor_position[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};	// 存放12个绳驱电机的速度控制指令




// USART3中断处理函数
void USART3_IRQHandler(void)
{  					
		OSIntEnter();

		// 清除溢出标志
		if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)//防止接受数据太快导致溢出
		{  	
				//清除中断标志    		 
				(void)USART3->SR;   
				(void)USART3->DR;
				return;			
		}
		
    // 检查接收中断标志
    if (USART_GetITStatus(USART3, USART_IT_RXNE)) {
        uint8_t byte = USART_ReceiveData(USART3); // 读取接收到的字节

        // 检查数据包开头0xAA55
        if (byte == 0xAA && header_count == 0) {
            header_count++;
        } else if (byte == 0x55 && header_count == 1) {
            header_count = 0;
            rx_index = 0;
            rx_buffer[rx_index++] = 0xAA;
            rx_buffer[rx_index++] = 0x55;
        } else if (rx_index > 0) {
            rx_buffer[rx_index++] = byte;

            // 第四个字节是数据长度
            if (rx_index == 4) {
                expected_length = byte;
            }

            // 当接收到预期数量的字节后
            if (rx_index == expected_length + 4) { // +4 是因为包括开头的2字节、控制器地址、长度字节
                // 计算接收到的数据的CRC-16 modbus校验码
                uint16_t calculated_crc = calc_crc16_modbus(rx_buffer, rx_index - 2); // 不包括CRC-16字段

                // 提取接收到的CRC-16 modbus校验码
                uint16_t received_crc = (rx_buffer[rx_index - 2] << 8) | rx_buffer[rx_index - 1];

                // 对比校验码
                if (calculated_crc == received_crc) {
                    // 校验成功，提取数据段
                    // 数据段从索引4开始，长度为(expected_length)
                    // 在这里处理数据段
										controller_address = rx_buffer[2]; // 控制器地址
										function_code = rx_buffer[4];      // 功能码
										uint8_t motor_count = rx_buffer[5];        // 所控制的电机数量
									
										if(function_code == 0x31){			// 表示对绳驱电机的控制		
												// 解析电机信息
												uint16_t offset = 6; // 电机信息从索引6开始
												uint8_t motor_address;	// 定义电机地址字段
												for (uint8_t i = 0; i < motor_count; ++i) {
														motor_address = rx_buffer[offset++]-1;	// 取得电机地址数据
														snake_motor_position[motor_address] = (rx_buffer[offset] << 24) | (rx_buffer[offset + 1] << 16) | (rx_buffer[offset + 2] << 8) | rx_buffer[offset + 3];
														offset += 4;
												}
										}
										
                } else {
                    // 校验失败，可能需要错误处理
										// 这里是串口输出计算出来的crc校验码
										Receive = (calculated_crc >> 8) & 0xFF;
										TX2_Transmit_Start();
										Receive = calculated_crc & 0xFF;
										TX2_Transmit_Start();
                }

                // 重置接收缓冲区索引
                rx_index = 0;
            }
        }
    }

    // 清除接收中断标志
    USART_ClearFlag(USART3, USART_FLAG_RXNE);
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		OSIntExit();
}


// 数据发送函数
void TX2_Transmit_Start(void)
{
	USART_ClearFlag(USART3,USART_FLAG_TC);
	USART_SendData(USART3,Receive);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);//判断是否发送完成

}






////定时器2的回调函数
//void tmr2_callback(void *p_tmr, void *p_arg)
//{
////			flag_get_pos=0;  //没有数据变0
////      again_spin=0;
////	    time_again_flag=1;  
//}

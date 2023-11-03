#include "can1.h"
#include "can2.h"
#include "uart1.h"
#include "uart3.h"
#include "task.h"
#include "delay.h"
//#include "uart8.h"
#include "gpio.h"
//#include "pstwo.h"
#include "pwm.h"
#include "includes.h"
int16_t Adc_Volt,limit_shift,cap_error=1; //can通讯所需



void TX2_USART3_Init(void);

void Chasis_task(void *p_arg)
{
		OS_ERR err;
		p_arg = p_arg; 
		for(int i = 0; i < 12; i++){
			snake_motor_position_control[i] = 0;
		}
		snake_motor_position_control[11] = 10;
		delay_us(200);
		while(1)
	{  		
			if(time_counter < 601){
				time_counter = time_counter +1;	
			}
			// send commands to 12 CAN module to control the position of snake motors
			// 发送can总线的电机位置控制指令
			for(uint8_t i=0; i < 12; i++){
				uint8_t bytes[4];  // 用于存储4个字节的数组
				uint8_t can_id = i + 1;

				if(reset_control == 0){
					motorEnable(can_id,0x06,0x01,0x0F,0x00,0x00,0x00,0x01); // 工作模式为1
					delay_us(100);
					// 分解 int32_t 变量为4个字节
					bytes[0] = ((snake_motor_position_control[i]) >> 24) & 0xFF;  // 最高有效字节 (MSB)
					bytes[1] = ((snake_motor_position_control[i]) >> 16) & 0xFF;  // 次高有效字节
					bytes[2] = ((snake_motor_position_control[i]) >> 8) & 0xFF;   // 次低有效字节
					bytes[3] = (snake_motor_position_control[i]) & 0xFF;          // 最低有效字节 (LSB)
					if(time_counter >= 500){
						setMotorTargetPosition(can_id,0x06,0x01,0x0A,bytes[0],bytes[1],bytes[2],bytes[3]); //设定目标位置值，32位有符号数；
					}						
				}

				delay_us(100);
				// reading the encorder
				readSnakeEncorder(can_id,0x02,0x03,0x07);
			}
			

			
/************************************************************* 
			send motors encorder position to the master device
			发送电机编码器反馈值给上位机
***************************************************************/
			uint8_t packet[79] = {0}; 			
			// header: 0xAA55
			packet[0] = 0xAA;
			packet[1] = 0x55;
			// id: 0x01
			packet[2] = 0x01;
			// data length: 0x4B (1 + 1 + 12 * 5 + 3*3 + 2 + 2) = 75
			packet[3] = 0x4B;
			// function code: 0x41
			packet[4] = 0x41;
			// motors number: 0x10 (16)
			packet[5] = 0x10;
			// sensors data
			int idx = 6;
			for (int i = 0; i < 12; ++i) {
					packet[idx++] = i + 1;  // encorder addresss
					// encorder data
					packet[idx++] = ((currentPosition_snake[i]) >> 24) & 0xFF;  
					packet[idx++] = ((currentPosition_snake[i]) >> 16) & 0xFF;
					packet[idx++] = ((currentPosition_snake[i]) >> 8) & 0xFF;
					packet[idx++] = ((currentPosition_snake[i]) 		) & 0xFF;
					
			}
			// GM6020
			packet[idx++] =13;
			packet[idx++] = (GM6020_absolute_position >> 8) & 0xFF;
			packet[idx++] = GM6020_absolute_position & 0xFF;
			// C610
			packet[idx++] =14;
			packet[idx++] = (C610_absolute_position >> 8) & 0xFF;
			packet[idx++] = C610_absolute_position & 0xFF;
			// STS3032
			packet[idx++] =15;
			packet[idx++] = (gripper_sts3032_position_control >> 8) & 0xFF;
			packet[idx++] = gripper_sts3032_position_control & 0xFF;
			// Reset state
			packet[idx++] =16;
			packet[idx++] = reset_control & 0xFF;
			
			// CRC-16 modbus
			uint16_t crc = calc_crc16_modbus(packet, idx);
			packet[idx++] = (crc >> 8) & 0xFF; 
			packet[idx++] = crc & 0xFF;
			// send back snake motor encorders to the master device
			for(uint8_t i=0;i< sizeof(packet);i++){
				// send to the usart3
				USART_ClearFlag(USART3,USART_FLAG_TC);
				USART_SendData(USART3,packet[i]);
				while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);//判断是否发送完成
			}
			
			OSTimeDly(8,OS_OPT_TIME_PERIODIC,&err); //延时8ms	
	}
	
}


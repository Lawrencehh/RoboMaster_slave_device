#include "can1.h"
#include "can2.h"
#include "uart1.h"
#include "uart3.h"
#include "task.h"
#include "delay.h"
#include "Judge_System.h"
#include "gpio.h"
//#include "pstwo.h"
#include "pwm.h"
#include "includes.h"
int angle_3=150;
int angle_2=90;
int angle_11=135;

int16_t output_202=0;
int16_t output_203=0;



int16_t target_velo_202=0;
int16_t target_velo_203=0;
int16_t target_velo_204=0;

int16_t Adc_Volt,limit_shift,cap_error=1;

int16_t error1=0,error2=0,error3=0,error4=0;
float anger_Vw[2];
u8 flag_Vw=0,flag_chasis_mode_shift=0,count_chasis_mode_shift=1;
uint32_t sum_chasis_output;
u8 flag_chasis_turnback=0,flag_ahead=0,flag_capacitance_switch=1;
int16_t last_chasis_mode;
u8 flag_chasis_spin_limit_off=1,storage_swtich;
u8 flag_storage_on=0;

static uint16_t ctlValue = 10 * 100;

enum mode
{
	spin_mode,
	follow_mode,
	uniaxial_mode,
	lock_mode        //锁定模式 打符
}chasis_mode=follow_mode;



void TX2_USART3_Init(void);
void TX2_Transmit_Start(void);
void Chasis_task(void *p_arg)
{
		OS_ERR err;
		p_arg = p_arg; 
	
		while(1)
	{  
//    Chasis_ESC_Send(500,0,500,0);
//		clock_900++;
		//id1 正退 id2正退 id3正退
//		if(ENABLE_yes==1)
//		{
//			setMotorTargetSpeed(0x01,0x06,0x01,0x09,0x00,0x00,0x2A,0xAA);
//			setMotorTargetPosition(0x01,0x06,0x01,0x0A,0x00,0x00,0x00,0x00);
//		  setMotorTargetPosition_2(0x02,0x06,0x01,0x0A,0xff,0x9b,0xff,0xff);
//		}
		
			// send commands to 12 CAN module to control the position of snake motors
			for(uint8_t i=0; i < 12; i++){
				uint8_t bytes[4];  // 用于存储4个字节的数组
				uint8_t can_id = i + 1;
				// 分解 int32_t 变量为4个字节
				bytes[0] = (snake_motor_position[i] >> 24) & 0xFF;  // 最高有效字节 (MSB)
				bytes[1] = (snake_motor_position[i] >> 16) & 0xFF;  // 次高有效字节
				bytes[2] = (snake_motor_position[i] >> 8) & 0xFF;   // 次低有效字节
				bytes[3] = snake_motor_position[i] & 0xFF;          // 最低有效字节 (LSB)
				setMotorTargetPosition(can_id,0x06,0x01,0x0A,bytes[0],bytes[1],bytes[2],bytes[3]); //设定目标位置值，32位有符号数；
				delay_us(100);
				// reading the encorder
				readEncorder(can_id,0x02,0x03,0x07);
			}
			
			  uint8_t packet[68] = {0}; 
				
				// header: 0xAA55
				packet[0] = 0xAA;
				packet[1] = 0x55;

				// id: 0x01
				packet[2] = 0x01;

				// data length: 0x40 (1 + 1 + 12 * 5 + 2)
				packet[3] = 0x40;

				// function code: 0x41
				packet[4] = 0x41;

				// motors number: 0x0C (12)
				packet[5] = 0x0C;

				// sensors data
				int idx = 6;
				for (int i = 0; i < 12; ++i) {
						packet[idx++] = i + 1;  // encorder addresss
						// encorder data
						packet[idx++] = (currentPosition[i] >> 24) & 0xFF;  
						packet[idx++] = (currentPosition[i] >> 16) & 0xFF;
						packet[idx++] = (currentPosition[i] >> 8) & 0xFF;
						packet[idx++] = (currentPosition[i] 		) & 0xFF;
						
				}

				// CRC-16 modbus
				uint16_t crc = calc_crc16_modbus(packet, idx);
				packet[idx++] = (crc >> 8) & 0xFF; 
				packet[idx++] = crc & 0xFF;

				
				

			for(uint8_t i=0;i< sizeof(packet);i++){
				// send to the usart3
				USART_ClearFlag(USART3,USART_FLAG_TC);
				USART_SendData(USART3,packet[i]);
				while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);//判断是否发送完成
			}
				
			
				
			
			OSTimeDly(8,OS_OPT_TIME_PERIODIC,&err); //延时8ms	
	}
	
}


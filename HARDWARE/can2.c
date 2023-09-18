#include "can2.h"
#include "can1.h"
#include "task.h"
#include "delay.h"
#include "includes.h"

//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif

Chasis_ID_t_Can2 Firction_201_t,Firction_202_t,Bodan_203_t,Bodan_204_t;  //203辅助 2006电机
int32_t phase_bodan,phase[2],phase_mid;
int16_t round_bodan=0;
Quaternion_t Quaternion;
Euler_angle_t Euler_angle;
angular_velocity_t angular_velocity;
acceleration_t acceleration;
magnetic_field_t magnetic_field;
float anger_bodan,anger_bodan_min;
float anger_pitch1,anger_yaw1;


float pitch_angle[2];
int16_t ecd_206_rate;
int16_t round_206;
void CAN2_Init(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef 	     gpio;
    NVIC_InitTypeDef   	   nvic;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);    //开启复用时钟，无论CAN1还是CAN2都要有这句
	
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);  //IO引脚复用
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio);
		
    nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
	  CAN_StructInit(&can);	
		can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_2tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 6;   //CAN BaudRate 42/(1+2+4)/6=1Mbps
		
	  CAN_DeInit(CAN2);
		CAN_Init(CAN2, &can);
		
	  can_filter.CAN_FilterNumber = 14;
    can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh = 0x0000;
    can_filter.CAN_FilterIdLow = 0x0000;
    can_filter.CAN_FilterMaskIdHigh = 0x0000;
    can_filter.CAN_FilterMaskIdLow = 0x0000;
    can_filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
    can_filter.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&can_filter);
		
		CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);  //使能发送中断
		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
}

unsigned char can2_tx_success_flag = 0;     //发送成功标志位
void CAN2_TX_IRQHandler(void)
{
		OSIntEnter();
    if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)
    {
        CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
        can2_tx_success_flag=1;
    }
		OSIntExit(); 
}

void CAN2_RX0_IRQHandler(void)
{	
    CanRxMsg rx_message;
    OSIntEnter();
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
		{
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx_message); 
			
			switch (rx_message.StdId)
        {
					case 0x201:  
					{
						Firction_201_t.phrase = (rx_message.Data[0]<<8)|rx_message.Data[1];
						Firction_201_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Firction_201_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						Firction_201_t.temper = rx_message.Data[6];
					}break;
					case 0x202: 
					{
						Firction_202_t.phrase = (rx_message.Data[0]<<8)|rx_message.Data[1];
						Firction_202_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Firction_202_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						Firction_202_t.temper = rx_message.Data[6];
					}break;
					case 0x401: 
					{
						Euler_angle.yaw = 0.0001*((rx_message.Data[0]<<8)|rx_message.Data[1]);
						Euler_angle.pitch = 0.0001*((rx_message.Data[2]<<8)|rx_message.Data[3]);
						Euler_angle.roll = 0.0001*((rx_message.Data[4]<<8)|rx_message.Data[5]);
					}break;
					case 0x402: 
					{
						Firction_202_t.phrase = (rx_message.Data[0]<<8)|rx_message.Data[1];
						Firction_202_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Firction_202_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						Firction_202_t.temper = rx_message.Data[6];
					}break;
					case 0x403: 
					{
						Firction_202_t.phrase = (rx_message.Data[0]<<8)|rx_message.Data[1];
						Firction_202_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Firction_202_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						Firction_202_t.temper = rx_message.Data[6];
					}break;
					case 0x404: 
					{
						Firction_202_t.phrase = (rx_message.Data[0]<<8)|rx_message.Data[1];
						Firction_202_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Firction_202_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						Firction_202_t.temper = rx_message.Data[6];
					}break;
					case 0x405: 
					{
						Firction_202_t.phrase = (rx_message.Data[0]<<8)|rx_message.Data[1];
						Firction_202_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Firction_202_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						Firction_202_t.temper = rx_message.Data[6];
					}break;
					default:
						break;
				}
    }
		OSIntExit();
}

void Chasis_ESC_Send_Can2(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (u8)(current_201 >> 8);
    tx_message.Data[1] = (u8)current_201;
    tx_message.Data[2] = (u8)(current_202 >> 8);
    tx_message.Data[3] = (u8)current_202;
		tx_message.Data[4] = (u8)(current_203 >> 8);
    tx_message.Data[5] = (u8)current_203;
    tx_message.Data[6] = (u8)(current_204 >> 8);
    tx_message.Data[7] = (u8)current_204;
    
    CAN_Transmit(CAN2,&tx_message);
}

void Gimbal_ESC_Send_Can2(int16_t current_205,int16_t current_206,int16_t current_207,int16_t current_208)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = 0x1ff;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (u8)(current_205 >> 8);
    tx_message.Data[1] = (u8)current_205;
    tx_message.Data[2] = (u8)(current_206 >> 8);
    tx_message.Data[3] = (u8)current_206;
    tx_message.Data[4] = (u8)(current_207 >> 8);
    tx_message.Data[5] = (u8)current_207;
    tx_message.Data[6] = (u8)(current_208 >> 8);
    tx_message.Data[7] = (u8)current_208;
    
    CAN_Transmit(CAN2,&tx_message);
}

void Gimbal_ESC_Send_Can2_yaw(int16_t current_205,int16_t current_206,int16_t current_207,int16_t current_208)   //拨码错了 贼难受
{
    CanTxMsg tx_message;
  
    tx_message.StdId = 0x2ff;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (u8)(current_205 >> 8);
    tx_message.Data[1] = (u8)current_205;
    tx_message.Data[2] = (u8)(current_206 >> 8);
    tx_message.Data[3] = (u8)current_206;
    tx_message.Data[4] = (u8)(current_207 >> 8);
    tx_message.Data[5] = (u8)current_207;
    tx_message.Data[6] = (u8)(current_208 >> 8);
    tx_message.Data[7] = (u8)current_208;
    
    CAN_Transmit(CAN2,&tx_message);
}


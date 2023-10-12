#include "can1.h"
#include "task.h"
#include "delay.h"
//#include "uart1.h"
#include "uart3.h"
//#include "pwm.h"
//#include "imu.h"
//#include "gpio.h"
#include "includes.h"
/*************************************************************************						
										初始化CAN1，1M波特率
*************************************************************************/
//////////////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif

// 这些全局变量用于在整个代码中存储和传递信息。
int flag;
int16_t receive[4];
int16_t adc_U;
int32_t currentPosition_snake[12]={0,0,0,0,0,0,0,0,0,0,0,0};


// 计算GM6020绝对位置的参数
int32_t GM6020_absolute_position = 0;  // 绝对位置
int16_t GM6020_last_raw_position;  // 上一次的原始位置（0-360）
int16_t GM6020_current_raw_position;  // 当前的原始位置（0-360）
int32_t GM6020_rotation_count = 0;  // 旋转计数（每完成一圈增加1或减少1）
// 计算C610绝对位置的参数
int32_t C610_absolute_position = 0;  // 绝对位置
int16_t C610_last_raw_position;  // 上一次的原始位置（0-360）
int16_t C610_current_raw_position;  // 当前的原始位置（0-360）
int32_t C610_rotation_count = 0;  // 旋转计数（每完成一圈增加1或减少1）

// 这个函数用于初始化CAN1接口。它设置了GPIO、NVIC（中断控制器）、CAN过滤器等。
void CAN1_Init(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef 	     gpio;
    NVIC_InitTypeDef   	   nvic;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);    //开启复用时钟，无论CAN1还是CAN2都要有这句
	
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);  //IO引脚复用
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &gpio);
		
    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
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
		
	  CAN_DeInit(CAN1);
		CAN_Init(CAN1, &can);
		
	  can_filter.CAN_FilterNumber = 0;
    can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh = 0x0000;
    can_filter.CAN_FilterIdLow = 0x0000;
    can_filter.CAN_FilterMaskIdHigh = 0x0000;
    can_filter.CAN_FilterMaskIdLow = 0x0000;
    can_filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
    can_filter.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&can_filter);
		
		CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);  //使能发送中断
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
}


/*************************************************************************
                          CAN1_TX_IRQHandler
描述：CAN1的发送中断函数
*************************************************************************/
unsigned char can_tx_success_flag = 0;     //发送成功标志位
void CAN1_TX_IRQHandler(void)
{
		OSIntEnter();
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
        can_tx_success_flag=1;
    }
		OSIntExit(); 
}
	 
GripperMotor_ID_t GripperMotor_205_t; // 手部的GM6020电机
GripperMotor_ID_t GripperMotor_201_t; // 手部的C610电机
int32_t phase_2006,phase_2006_bodan[2],phase_mid_2006,round_bodan_2006=0;
int32_t phase_6020[2],round_6020_yaw=0;
float anger_bodan_2006,phase_6020_yaw;
float angle_G2C;//云台与底盘夹角
/*************************************************************************
                          CAN1_RX0_IRQHandler
描述：CAN1的接收中断函数
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{	
    CanRxMsg rx_message;
    OSIntEnter();
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
		{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message); 
			
				switch (rx_message.StdId)
        {
					case 0x01:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[0]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					case 0x02:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[1]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					case 0x03:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[2]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					case 0x04:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[3]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					case 0x05:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[4]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					case 0x06:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[5]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					case 0x07:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[6]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					case 0x08:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[7]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					case 0x09:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[8]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					case 0x0A:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[9]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					case 0x0B:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[10]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					case 0x0C:   //id
					{
						if(rx_message.Data[0]==0x04){
							currentPosition_snake[11]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]);
						}
					}break;	
					
					//手部电机
					case 0x00000205:				// GM6020 回传数据，ID=1	
					{
						GripperMotor_205_t.position = ((rx_message.Data[0]<<8)|rx_message.Data[1]) *360/8192;;
						GripperMotor_205_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						GripperMotor_205_t.current = (rx_message.Data[4]<<8)|rx_message.Data[5];
						GripperMotor_205_t.temperature = rx_message.Data[6];
					}break;
					
					case 0x00000201:				// C610 回传数据，ID=1	
					{
						GripperMotor_201_t.position = ((rx_message.Data[0]<<8)|rx_message.Data[1])*360/8192;
						GripperMotor_201_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						GripperMotor_201_t.current = (rx_message.Data[4]<<8)|rx_message.Data[5];
						GripperMotor_201_t.temperature = rx_message.Data[6];
					}break;

					default:
						break;
				}
				
	

    }
		OSIntExit();
}


// 这个函数用于发送底盘电机的电流值。
void Chasis_ESC_Send(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204)
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
    
    CAN_Transmit(CAN1,&tx_message);
}




// 发送读取绳驱电机编码器指令
void readSnakeEncorder(u8 STdId,u8 dlc,u8 D0,u8 D1)	
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
    CAN_Transmit(CAN1,&tx_message);
}

// GM6020关节电机
void GM6020_Can_Send_Msg(int16_t current_1,int16_t current_2,int16_t current_3,int16_t current_4)
{
    CanTxMsg tx_message; 
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;   
    tx_message.Data[0] = (u8)(current_1 >> 8);
    tx_message.Data[1] = (u8)current_1;
    tx_message.Data[2] = (u8)(current_2 >> 8); 
    tx_message.Data[3] = (u8)current_2;
	  tx_message.Data[4] = (u8)(current_3 >> 8);
    tx_message.Data[5] = (u8)current_3;
    tx_message.Data[6] = (u8)(current_4 >> 8); 
    tx_message.Data[7] = (u8)current_4;
    CAN_Transmit(CAN1,&tx_message);
}

// C610电机
void C610_Can_Send_Msg(int16_t current_1,int16_t current_2,int16_t current_3,int16_t current_4)
{
    CanTxMsg tx_message; 
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;   
    tx_message.Data[0] = (u8)(current_1 >> 8);
    tx_message.Data[1] = (u8)current_1;
    tx_message.Data[2] = (u8)(current_2 >> 8); 
    tx_message.Data[3] = (u8)current_2;
	  tx_message.Data[4] = (u8)(current_3 >> 8);
    tx_message.Data[5] = (u8)current_3;
    tx_message.Data[6] = (u8)(current_4 >> 8); 
    tx_message.Data[7] = (u8)current_4;
    CAN_Transmit(CAN1,&tx_message);
}

// 设置电机使能
void motorEnable(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}


// 设置电机目标位置
void setMotorTargetPosition(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}

// 读取电机电流
void readMotorCurrentValue(u8 STdId,u8 dlc,u8 D0,u8 D1)
{

    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
    CAN_Transmit(CAN1,&tx_message);
}

// 这个函数用于发送底盘电机的速度目标值。
void setMotorTargetSpeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)//报文发送	
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}



// 设置电机目标电流值
void setMotorTargetCurrent(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}

// 设置电机目标加速加速度
void setMotorTargetAcspeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)	
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}

// 设置电机目标减速加速度
void setMotorTargetDespeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)	
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}

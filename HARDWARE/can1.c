#include "can1.h"
#include "task.h"
#include "delay.h"
//#include "uart1.h"
//#include "uart3.h"
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

int flag;
int16_t receive[4];
int16_t adc_U;

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
	    
Chasis_ID_t Chasis_201_t,Chasis_202_t,Chasis_203_t,Chasis_204_t,Bodan_208_t;
Gimbal_ID_t Gimbal_205_t,Gimbal_206_t,Gimbal_207_t,Gimbal_208_t;
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
					case 0x141:   
					{
						if(rx_message.Data[0]==0xA2){
						Chasis_201_t.phrase= (rx_message.Data[6]<<8)|rx_message.Data[7];
						Chasis_201_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Chasis_201_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						Chasis_201_t.temper = rx_message.Data[6];
						}
					}break;
					case 0x31:   
					{
						Chasis_202_t.phrase = (rx_message.Data[0]<<8)|rx_message.Data[1];
						Chasis_202_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Chasis_202_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						Chasis_202_t.temper = rx_message.Data[6];
					}break;
					case 0x203:   
					{
						Chasis_203_t.phrase = (rx_message.Data[0]<<8)|rx_message.Data[1];
						Chasis_203_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Chasis_203_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						Chasis_203_t.temper = rx_message.Data[6];
					}break;
					case 0x204:   
					{
						Chasis_204_t.phrase = ((rx_message.Data[0]<<8)|rx_message.Data[1])*0.044f;
						Chasis_204_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Chasis_204_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						Chasis_204_t.temper = rx_message.Data[6];
					}break;
          case 0x205:   
					{
						Gimbal_205_t.phrase = (rx_message.Data[0]<<8)|rx_message.Data[1];
//						phase_6020[0]=phase_6020[1];
//						phase_6020[1]=Gimbal_205_t.phrase;
//						if(phase_6020[1]-phase_6020[0]>7000)round_6020_yaw--;
//						if(phase_6020[1]-phase_6020[0]<-7000)round_6020_yaw++;
//						phase_6020_yaw=(round_6020_yaw*8192+phase_6020[1])*0.044f;
						Gimbal_205_t.real_elecrent = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Gimbal_205_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						this_angle_Yaw =Gimbal_205_t.phrase * 0.044f;
//						if(this_angle_Yaw>=230){this_angle_Yaw=this_angle_Yaw-360.0f;}
//		        else{this_angle_Yaw=this_angle_Yaw;}
					}break;					
					case 0x206:					
					{
						Gimbal_206_t.phrase = (rx_message.Data[0]<<8)|rx_message.Data[1];
						Gimbal_206_t.real_elecrent = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Gimbal_206_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
						this_angle_Pitch =Gimbal_206_t.phrase * 0.044f;
					}break;
					case 0x208:
					{
						Bodan_208_t.phrase = (rx_message.Data[0]<<8)|rx_message.Data[1];
						phase_2006_bodan[0]=phase_2006_bodan[1];
						phase_2006_bodan[1]=Bodan_208_t.phrase*360/8192;
           if((phase_2006_bodan[1]-phase_2006_bodan[0])>300)round_bodan_2006++;
           if((phase_2006_bodan[1]-phase_2006_bodan[0])<-300)round_bodan_2006--;	
           phase_mid_2006=round_bodan_2006*360 + phase_2006_bodan[1];
           anger_bodan_2006=phase_mid_2006/36;	
					if(target_anger_bodan_208>30000 || target_anger_bodan_208<-30000)
						{
							target_anger_bodan_208=0;
							phase_mid_2006=0;
							phase_2006_bodan[0]=0;
							phase_2006_bodan[1]=0;
							anger_bodan_2006=0;
							round_bodan_2006=0;
						}
						Bodan_208_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
						Bodan_208_t.elecrent = (rx_message.Data[4]<<8)|rx_message.Data[5];
					}break;
					
					case 0x102:
					{
			      receive[0]=(rx_message.Data[0]<<8)|rx_message.Data[1];
						Adc_Volt=receive[0];
		       	receive[1]=(rx_message.Data[2]<<8)|rx_message.Data[3];
						cap_error=receive[1];
		       	receive[2]=(rx_message.Data[4]<<8)|rx_message.Data[5];
		       	receive[3]=(rx_message.Data[6]<<8)|rx_message.Data[7];
		      	 adc_U=receive[1];
					}break;
					default:
						break;
				}
    }
		OSIntExit();
}

void Chasis_ESC_Send(int16_t current_201,uint16_t ctlValue)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = 0x141;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
   // tx_message.Data[0] = (u8)(current_201 >> 8);
	  tx_message.Data[0] = (u8) current_201;
		tx_message.Data[1] =(u8) 0x00;
    tx_message.Data[2] = (u8)0x00;
    tx_message.Data[3] = (u8) 0x00;
		tx_message.Data[4] = *((uint8_t *)&ctlValue + 0);
    tx_message.Data[5] = *((uint8_t *)&ctlValue + 1);
    tx_message.Data[6] = (u8) 0x00;
    tx_message.Data[7] = (u8) 0x00;
    
    CAN_Transmit(CAN1,&tx_message);
}

void Gimbal_ESC_Send(int16_t current_205,int16_t current_206,int16_t current_208)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (u8)(current_205 >> 8);
    tx_message.Data[1] = (u8)current_205;
    tx_message.Data[2] = (u8)(current_206 >> 8);
    tx_message.Data[3] = (u8)current_206;
	  tx_message.Data[6] = (u8)(current_208 >> 8);
    tx_message.Data[7] = (u8)current_208;
    
    CAN_Transmit(CAN1,&tx_message);
}



void Gimbal_ESC_Send_minpitch(int16_t current_205,int16_t current_206,int16_t current_207,int16_t current_208)
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
    
    CAN_Transmit(CAN1,&tx_message);
}


void Can_Send_Msg(int16_t current_1,int16_t current_2,int16_t current_3,int16_t current_4)
{
    CanTxMsg tx_message; 
    tx_message.StdId = 0x101;
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



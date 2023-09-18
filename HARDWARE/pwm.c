#include "pwm.h"
#include "delay.h"
int16_t angle1=160,angle2=180,angle3=96,angle4=60;
int16_t angle=130;
int angle_1=115;

void TIM2_PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
		TIM_OCInitTypeDef ocx;
		GPIO_InitTypeDef gpio;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2); 
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2); 
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2); 
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2); 
		
		gpio.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; 
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA,&gpio); 
	
    tim.TIM_Prescaler = 900-1;        //90M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;
    TIM_TimeBaseInit(TIM2,&tim);
		TIM_Cmd(TIM2, ENABLE);
	
		ocx.TIM_OCMode=TIM_OCMode_PWM1;
    ocx.TIM_OutputState=TIM_OutputState_Enable; 
    ocx.TIM_Pulse=0;
    ocx.TIM_OCPolarity=TIM_OCPolarity_High;

		TIM_OC1Init(TIM2,&ocx);
		TIM_OC2Init(TIM2,&ocx);
		TIM_OC3Init(TIM2,&ocx);
		TIM_OC4Init(TIM2,&ocx);
    TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
		TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
		TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
		TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
   
    TIM_ARRPreloadConfig(TIM2,ENABLE);
    TIM_Cmd(TIM2,ENABLE);
		
		  TIM2->CCR1=100; //RIGHT 120  MIN50
  		TIM2->CCR2=100;  //LEFT  120  MIN  70 T
  		TIM2->CCR3=100;   // 60 
   		TIM2->CCR4=100;
		
}

void TIM4_PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
		TIM_OCInitTypeDef ocx;
		GPIO_InitTypeDef gpio;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); 
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4); 
	  GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4); 
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4); 
		
		gpio.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; 
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOD,&gpio); 
	
    tim.TIM_Prescaler = 900-1;        //90M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;
    TIM_TimeBaseInit(TIM4,&tim);
		TIM_Cmd(TIM4, ENABLE);
	
		ocx.TIM_OCMode=TIM_OCMode_PWM1;
    ocx.TIM_OutputState=TIM_OutputState_Enable; 
    ocx.TIM_Pulse=0;
    ocx.TIM_OCPolarity=TIM_OCPolarity_High;

		TIM_OC1Init(TIM4,&ocx);
		TIM_OC2Init(TIM4,&ocx);
		TIM_OC3Init(TIM4,&ocx);
		TIM_OC4Init(TIM4,&ocx);
    TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
		TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
		TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
		TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
   
    TIM_ARRPreloadConfig(TIM4,ENABLE);
    TIM_Cmd(TIM4,ENABLE);
		

  	TIM4->CCR1=150;//UP 120 MIN 130
//		TIM4->CCR2=angle2;
//		TIM4->CCR3=angle3;
//		TIM4->CCR4=angle_1;
//		
}
void TIM5_PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
		TIM_OCInitTypeDef ocx;
		GPIO_InitTypeDef gpio;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
		GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5); 
		GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5); 
	  GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5); 
		
		
		gpio.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_11|GPIO_Pin_10; 
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOH,&gpio); 
	
    tim.TIM_Prescaler = 900-1;        //90M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;
    TIM_TimeBaseInit(TIM5,&tim);
		TIM_Cmd(TIM5, ENABLE);
	
		ocx.TIM_OCMode=TIM_OCMode_PWM1;
    ocx.TIM_OutputState=TIM_OutputState_Enable; 
    ocx.TIM_Pulse=0;
    ocx.TIM_OCPolarity=TIM_OCPolarity_High;

		TIM_OC1Init(TIM5,&ocx);
		TIM_OC2Init(TIM5,&ocx);
		TIM_OC3Init(TIM5,&ocx);
		
    TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
		TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
		TIM_OC3PreloadConfig(TIM5,TIM_OCPreload_Enable);
		
   
    TIM_ARRPreloadConfig(TIM5,ENABLE);
    TIM_Cmd(TIM5,ENABLE);
		
//    TIM5->CCR1=200; // (1)da forward//135 70
//		TIM5->CCR2=220;//2 BACK XIAO
//		TIM5->CCR3=220;//3
//		TIM5->CCR4=500;
		
}

void TIM8_PWM_Init(void)
{
	  TIM_TimeBaseInitTypeDef tim;
		TIM_OCInitTypeDef ocx;
		GPIO_InitTypeDef gpio;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	
		GPIO_PinAFConfig(GPIOI,GPIO_PinSource5,GPIO_AF_TIM8); 
		GPIO_PinAFConfig(GPIOI,GPIO_PinSource6,GPIO_AF_TIM8); 
	  GPIO_PinAFConfig(GPIOI,GPIO_PinSource7,GPIO_AF_TIM8); 
		GPIO_PinAFConfig(GPIOI,GPIO_PinSource2,GPIO_AF_TIM8); 
		
		gpio.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOI,&gpio); 
	
    tim.TIM_Prescaler = 1800-1;        //90M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;
    TIM_TimeBaseInit(TIM8,&tim);
		TIM_Cmd(TIM8, ENABLE);
	
		ocx.TIM_OCMode=TIM_OCMode_PWM1;
    ocx.TIM_OutputState=TIM_OutputState_Enable; 
    ocx.TIM_Pulse=0;
    ocx.TIM_OCPolarity=TIM_OCPolarity_High;

		TIM_OC1Init(TIM8,&ocx);
		TIM_OC2Init(TIM8,&ocx);
		TIM_OC3Init(TIM8,&ocx);
		TIM_OC4Init(TIM8,&ocx);
		
    TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);
		TIM_OC2PreloadConfig(TIM8,TIM_OCPreload_Enable);
		TIM_OC3PreloadConfig(TIM8,TIM_OCPreload_Enable);
		TIM_OC4PreloadConfig(TIM8,TIM_OCPreload_Enable);
   
    TIM_ARRPreloadConfig(TIM8,ENABLE);
    TIM_Cmd(TIM8,ENABLE);
		
		TIM_CtrlPWMOutputs(TIM8,ENABLE);
		
		
//		TIM8->CCR1=115;//1 max 190 back min 110
		
}




//void Heat_PWM_Init(void)  
//{
//	  TIM_TimeBaseInitTypeDef tim;
//		TIM_OCInitTypeDef ocx;
//		GPIO_InitTypeDef gpio;
//	
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
//	
//		GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3); 
//		
//		gpio.GPIO_Pin = GPIO_Pin_5; 
//		gpio.GPIO_Mode = GPIO_Mode_AF;
//		gpio.GPIO_Speed = GPIO_Speed_100MHz;	
//		gpio.GPIO_OType = GPIO_OType_PP; 
//		gpio.GPIO_PuPd = GPIO_PuPd_UP;
//		GPIO_Init(GPIOB,&gpio); 
//	
//    tim.TIM_Prescaler = 900-1;        //90M internal clock
//    tim.TIM_CounterMode = TIM_CounterMode_Up;
//    tim.TIM_ClockDivision = TIM_CKD_DIV1;
//    tim.TIM_Period = 2000;
//    TIM_TimeBaseInit(TIM3,&tim);
//		TIM_Cmd(TIM3, ENABLE);
//	
//		ocx.TIM_OCMode=TIM_OCMode_PWM1;
//    ocx.TIM_OutputState=TIM_OutputState_Enable;
//    ocx.TIM_OutputNState=TIM_OutputNState_Disable;
//    ocx.TIM_Pulse=0;
//    ocx.TIM_OCPolarity=TIM_OCPolarity_High;
//    ocx.TIM_OCNPolarity=TIM_OCPolarity_High;
//    ocx.TIM_OCIdleState=TIM_OCIdleState_Reset;
//    ocx.TIM_OCNIdleState=TIM_OCIdleState_Set;
//	
//		TIM_OC2Init(TIM3,&ocx);
//    TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
//   
//    TIM_ARRPreloadConfig(TIM3,ENABLE);
//    TIM_Cmd(TIM3,ENABLE);
//		

//}

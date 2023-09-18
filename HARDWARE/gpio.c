#include "gpio.h"

void User_GPIO_Init(void)
{
    GPIO_InitTypeDef  gpio;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
		//24V可控输出
  	gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;   
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_Speed = GPIO_Speed_25MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		GPIO_Init(GPIOH,&gpio); 
	
	  gpio.GPIO_Pin = GPIO_Pin_13;   
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_Speed = GPIO_Speed_25MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		GPIO_Init(GPIOG,&gpio);
	
		gpio.GPIO_Pin = GPIO_Pin_3| GPIO_Pin_4;   
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_Speed = GPIO_Speed_25MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		GPIO_Init(GPIOA,&gpio);
	
	  gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12;   
		gpio.GPIO_Mode = GPIO_Mode_IN;
		gpio.GPIO_Speed = GPIO_Speed_25MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		GPIO_Init(GPIOD,&gpio);
		
		gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;   
		gpio.GPIO_Mode = GPIO_Mode_IN;
		gpio.GPIO_Speed = GPIO_Speed_25MHz;	
		gpio.GPIO_OType = GPIO_OType_PP;  
		GPIO_Init(GPIOH,&gpio);
		
		gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2| GPIO_Pin_7| GPIO_Pin_5;   
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_Speed = GPIO_Speed_25MHz;	
		gpio.GPIO_OType = GPIO_OType_PP;  
		GPIO_Init(GPIOI,&gpio);
		
		gpio.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_2| GPIO_Pin_4|GPIO_Pin_5 |GPIO_Pin_1 ;   
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_Speed = GPIO_Speed_25MHz;	
		gpio.GPIO_OType = GPIO_OType_PP;  
		GPIO_Init(GPIOC,&gpio);
	
	 
		GPIO_SetBits(GPIOH,GPIO_Pin_4);   //4个可控24V输出
	  GPIO_SetBits(GPIOH,GPIO_Pin_2);
	  GPIO_SetBits(GPIOH,GPIO_Pin_3);
	  GPIO_SetBits(GPIOH,GPIO_Pin_5);
//		GPIO_SetBits(GPIOA,GPIO_Pin_4);
//	  GPIO_SetBits(GPIOI,GPIO_Pin_5);
		GPIO_ResetBits(GPIOC,GPIO_Pin_2); 
		GPIO_SetBits(GPIOG,GPIO_Pin_13);    //激光供电
}

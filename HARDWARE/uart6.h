#ifndef __UART6_H
#define __UART6_H

#include <stm32f4xx.h>

//void AHRS_USART6_Init(void);
void USART6_IRQHandler(void);
void AHRS_Transmit_Start(void);
void AHRS_Transmit_Stop(void);
void AHRS_Transmit_Stop_Magnetic_Steering(void);
extern int16_t AHRS_Yaw_angle; 
extern float Angle_Yaw;

#endif



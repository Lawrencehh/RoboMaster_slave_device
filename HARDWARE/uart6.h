#ifndef __UART6_H
#define __UART6_H

#include <stm32f4xx.h>
#include <stdint.h>

void Uart6_Flush(void);
int16_t Uart6_Read(void);
void USART6_Init(void);
void USART6_IRQHandler(void);
void USART6_Handler(void);
void Uart6_Send(uint8_t *buf , uint8_t len);
//void Servocontrol(void);
#endif



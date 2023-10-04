#ifndef __UART6_H
#define __UART6_H

#include <stm32f4xx.h>
#include <stdint.h>
// 有关STS3032的反馈参数
extern uint16_t STS3032_Pos;
extern int STS3032_Speed;
extern int STS3032_Load;
extern int STS3032_Voltage;
extern int STS3032_Temper;
extern int STS3032_Move;
extern int STS3032_Current;
void Uart6_Flush(void);
int16_t Uart6_Read(void);
void USART6_Init(void);
void USART6_IRQHandler(void);
void USART6_Handler(void);
void Uart6_Send(uint8_t *buf , uint8_t len);


#endif



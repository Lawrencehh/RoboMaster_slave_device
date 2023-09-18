#ifndef __USART6_H
#define __USART6_H	 
#include "sys.h"  
#include "stdio.h"

#define USART6_MAX_RECV_LEN		256				//最大接收缓存字节数
#define USART6_MAX_SEND_LEN		256				//最大发送缓存字节数
#define USART6_RX_EN 			1				      //0,不接收;1,接收.

extern u8  USART6_RX_BUF[USART6_MAX_RECV_LEN]; 		
extern u8  USART6_TX_BUF[USART6_MAX_SEND_LEN]; 		
extern vu16 USART6_RX_STA;   		//接收数据状态


void USART6_init(void);
void Usart6_Send(unsigned char *DataToSend ,u8 data_num);
#endif


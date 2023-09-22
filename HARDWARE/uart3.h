#ifndef __UART2_H
#define __UART2_H

#include <stm32f4xx.h>

void TX2_USART3_Init(void);
void USART3_IRQHandler(void);
void TX2_Transmit_Start(void);
float SXT_pitch(float target_pitch_position,float middle_pitch_position);
float SXT_yaw(float target_yaw_position,float middle_yaw_position);
void tmr2_callback(void *p_tmr, void *p_arg);

extern float Camera_Inc_Pitch,Camera_Inc_Yaw,Camera_Distance;
extern u8 NewData_flag;
extern int32_t snake_motor_position[12];
#endif

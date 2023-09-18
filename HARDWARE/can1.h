#ifndef __CAN1_H__
#define __CAN1_H__

#include <stm32f4xx.h>

void CAN1_Init(void);
void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void Chasis_ESC_Send(int16_t current_201,uint16_t ctlValue);
void Gimbal_ESC_Send(int16_t current_205,int16_t current_206,int16_t current_208);
void Gimbal_ESC_Send_minpitch(int16_t current_205,int16_t current_206,int16_t current_207,int16_t current_208);
void Gun_ESC_Send(int16_t current_201);
void Can_Send_Msg(int16_t current_1,int16_t current_2,int16_t current_3,int16_t current_4);

typedef struct
{
	int16_t phrase;
	int16_t velocity;
	int16_t elecrent;
	int8_t  temper;
}Chasis_ID_t;

typedef struct
{
	int16_t phrase;
	int16_t real_elecrent;
	int16_t elecrent;
}Gimbal_ID_t;


extern Chasis_ID_t Chasis_201_t,Chasis_202_t,Chasis_203_t,Chasis_204_t,Bodan_208_t;
extern Gimbal_ID_t Gimbal_205_t,Gimbal_206_t,Gimbal_207_t,Gimbal_208_t;

extern int flag;
extern int16_t receive[4];
extern int16_t adc_U,cap_error;
extern int32_t round_6020_yaw;
extern float phase_6020_yaw;

#endif

#ifndef __CAN1_H__
#define __CAN1_H__

#include <stm32f4xx.h>

void CAN1_Init(void);
void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void Chasis_ESC_Send(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204);
void setMotorTargetSpeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void motorEnable(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetSpeed_2(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void motorEnable_2(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void Gun_ESC_Send(int16_t current_201);
void setMotorTargetPosition(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetCurrent(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetAcspeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetDespeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetPosition_2(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void readMotorCurrentValue(u8 STdId,u8 dlc,u8 D0,u8 D1);
void readEncorder(u8 STdId,u8 dlc,u8 D0,u8 D1);

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
extern int ENABLE_yes;
extern int flag;
extern int16_t receive[4];
extern int16_t adc_U,cap_error;
extern int32_t round_6020_yaw;
extern float phase_6020_yaw;
extern int32_t currentPosition[12];
extern double currentSpeed;
extern double current;

#endif

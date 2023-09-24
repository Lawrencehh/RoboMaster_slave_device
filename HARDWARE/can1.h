#ifndef __CAN1_H__
#define __CAN1_H__

#include <stm32f4xx.h>

void CAN1_Init(void);
void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);

void Chasis_ESC_Send(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204);

void motorEnable(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetSpeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetPosition(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetCurrent(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetAcspeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetDespeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void readMotorCurrentValue(u8 STdId,u8 dlc,u8 D0,u8 D1);

void readSnakeEncorder(u8 STdId,u8 dlc,u8 D0,u8 D1); //读取绳驱电机的编码器值

void Can_Send_Msg(int16_t current_1,int16_t current_2,int16_t current_3,int16_t current_4); // 6020关节电机控制


typedef struct
{
	int16_t position;
	int16_t velocity;
	int16_t current;
	int8_t  temperature;
}GripperMotor_ID_t;


extern GripperMotor_ID_t GripperMotor_205_t; // 手部的GM6020电机
extern int ENABLE_yes;
extern int flag;
extern int16_t receive[4];
extern int16_t adc_U,cap_error;
extern int32_t round_6020_yaw;
extern float phase_6020_yaw;
extern int32_t currentPosition_snake[12];
extern double currentSpeed;
extern double current;

extern int16_t GM6020_last_raw_position;  // 上一次的原始位置（0-8191）
extern int16_t GM6020_current_raw_position;  // 当前的原始位置（0-8191）

#endif

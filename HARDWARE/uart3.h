#ifndef __UART2_H
#define __UART2_H

#include <stm32f4xx.h>

void TX2_USART3_Init(void);
void USART3_IRQHandler(void);
void TX2_Transmit_Start(void);
uint16_t calc_crc16_modbus(uint8_t *data, uint16_t length);

float SXT_pitch(float target_pitch_position,float middle_pitch_position);
float SXT_yaw(float target_yaw_position,float middle_yaw_position);
void tmr2_callback(void *p_tmr, void *p_arg);

extern float Camera_Inc_Pitch,Camera_Inc_Yaw,Camera_Distance;
extern u8 NewData_flag;
extern int32_t snake_motor_position_control[12];
extern const uint16_t crc16_table[256];

// gripper gm6020 的位置控制
extern int16_t gripper_gm6020_position_control;
// gripper c610 的位置控制
extern int16_t gripper_c610_position_control;
// gripper sts3032 的位置控制
extern int16_t gripper_sts3032_position_control;
extern int16_t last_sts3032_control_value;
// 传感器清零控制
extern int16_t reset_control;
extern int16_t last_reset_control;
extern int32_t snake_motor_position_reset_offset[12];
extern int16_t gripper_gm6020_position_reset_offset;
extern int16_t gripper_c610_position_reset_offset;
extern int16_t gripper_sts3032_position_reset_offset;


#endif

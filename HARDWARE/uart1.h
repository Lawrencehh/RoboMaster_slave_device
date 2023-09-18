#ifndef __UART1_H
#define __UART1_H

#include <stm32f4xx.h>

typedef __packed struct
{
	int8_t switch_l;
	int8_t switch_r;
	uint8_t mouse_l;
	uint8_t mouse_r;
	int16_t right_x;
	int16_t right_y;
	int16_t left_x;
	int16_t left_y;
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint16_t  key;
}Tel_Ctrl_t;

extern Tel_Ctrl_t TelCtrlData;
extern u8 TeleCtrl_Rev_OK;
extern int16_t Vy,Vx,Vw;
extern float V_mousex,V_mousey; 
extern int16_t flag_bodan_finish;
extern int32_t keyboard;

void TEL_USART1_Init(void);
void RemoteDataProcess(uint8_t *sbus_rx_buffer); 
void Tel_Control(void);

#define Keyboard_W  1
#define Keyboard_A  4
#define Keyboard_S  2
#define Keyboard_D  8
#define Keyboard_Q  64
#define Keyboard_E  128
#define Keyboard_R  256
#define Keyboard_F  512
#define Keyboard_G  1024
#define Keyboard_Z  2048
#define Keyboard_X  4096
#define Keyboard_C  8192
#define Keyboard_V  16384
#define Keyboard_B  32768
#define Keyboard_shift  16
#define Keyboard_ctrl  32
#define MID_angle  180           //底盘最中间的编码器角度值

//#define V_max  150

#endif


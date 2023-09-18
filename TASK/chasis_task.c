#include "can1.h"
#include "can2.h"
#include "uart1.h"
#include "task.h"
#include "delay.h"
#include "Judge_System.h"
#include "gpio.h"
//#include "pstwo.h"
#include "pwm.h"
#include "includes.h"
int angle_3=150;
int angle_2=90;
int angle_11=135;

int16_t output_202=0;
int16_t output_203=0;



int16_t target_velo_202=0;
int16_t target_velo_203=0;
int16_t target_velo_204=0;

int16_t Adc_Volt,limit_shift,cap_error=1;

int16_t error1=0,error2=0,error3=0,error4=0;
float anger_Vw[2];
u8 flag_Vw=0,flag_chasis_mode_shift=0,count_chasis_mode_shift=1;
uint32_t sum_chasis_output;
u8 flag_chasis_turnback=0,flag_ahead=0,flag_capacitance_switch=1;
int16_t last_chasis_mode;
u8 flag_chasis_spin_limit_off=1,storage_swtich;
u8 flag_storage_on=0;

static uint16_t ctlValue = 10 * 100;

enum mode
{
	spin_mode,
	follow_mode,
	uniaxial_mode,
	lock_mode        //锁定模式 打符
}chasis_mode=follow_mode;



void TX2_USART3_Init(void);
void TX2_Transmit_Start(void);
void Chasis_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg; 
	
  
		Chasis_ESC_Send(0xA7, ctlValue);//延时9ms

		
	
}


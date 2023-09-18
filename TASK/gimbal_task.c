#include "can1.h"
#include "can2.h"
#include "task.h"
#include "delay.h"
#include "pstwo.h"
#include "uart3.h"
#include "pwm.h"
#include "imu.h"
#include "gpio.h"
#include "includes.h"
#include "math.h"

float velo_206_output= 0.0f;       //pitch轴速度环函数输出值

int16_t output_205=0,output_206=0;
int16_t AHRS_Yaw_angle=0; 
int16_t flag_100D2=0,count_2006=0;   //2006计数

float target_angle_Pitch1=300.0f,target_angle_yaw1=180.0f;  //大枪管yaw pitch 角度
float last_bodan_anger;
float Eular[3],yaw_9050,pitch_9050;
int16_t round_count=0;
float Yaw_9050[2];
float target_anger_bodan_204;
int count_3508=0;

static uint16_t ctlValue = 10 * 100;

float this_angle_Pitch,target_angle_Pitch=190.0f,this_angle_Yaw;



int16_t target_velo_201=0;
int16_t output_201=0;
int16_t output_204=0;

int gun_count=0,i_6=100,i_8=100,i_13=150,i_14=100,i_16=100,i_100=0;
enum mode
{ 
	auto_mode,
	follow_mode,
	uniaxial_mode,
	buff_mode,
	buff_spin_mode
}gimbal_mode=follow_mode;


void Gimbal_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{  
		
//		TX2_Transmit_Start();

          OSTimeDly(10,OS_OPT_TIME_PERIODIC,&err); //延时10ms
  } 
}




#include "can2.h"
#include "can1.h"
#include "task.h"
#include "delay.h"
#include "uart1.h"
//#include "uart3.h"
#include "uart6.h"
#include "pwm.h"
#include "imu.h"
#include "gpio.h"
#include "includes.h"
#include "Judge_System.h"
u8 continue_flag=1;
int16_t target_V201_friction,target_V202_friction,target_V208_bodan,target_anger_bodan_208=0.0f;
int16_t output_201_firction=0,output_202_firction=0,output_208_bodan=0;
int16_t flag_bodan=0,flag_bodan_finish=1,flag_count=0,count_block=0,count_turn=0;
int16_t flag1=0,flag2=0;
int16_t count_bullet=0,amount_42mm=0;
int16_t error_bodan[2],error_bodan_min[2],count_error_bo=0,count_error=0,count_error_min=0;
u8 speed_ready=1;
int16_t count_j=0,count_jj[5],count_shift=0;
u8 flag_friction_start=0,flag_bo_finish=0;
int16_t error11,error22;
int16_t last_fire_mode;
u8 flag_V_shoot_mode_shift=0,count_shoot_mode_shift=1,flag_shoot_mode_shift=0;
int16_t friction_start=200;

enum mode
{
	low_speed_mode,
	middle_speed_mode,
	high_speed_mode
}shoot_speed_mode=middle_speed_mode;

enum Mode
{
	sigle_fire_mode,
	triple_fire_mode,
	burst_fire_mode
}shoot_mode=sigle_fire_mode;

void Gun_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{  

		OSTimeDly(10,OS_OPT_TIME_PERIODIC,&err); //延时10ms	
	}
} 
	




 











//定时器1的回调函数
void tmr1_callback(void *p_tmr, void *p_arg)
{
	continue_flag=1;		//定时器1执行次数加1
}

void tmr2_callback(void *p_tmr, void *p_arg)
{
//			flag_get_pos=0;  //没有数据变0
//      again_spin=0;
//	    time_again_flag=1;  
}



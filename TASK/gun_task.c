#include "can2.h"
#include "can1.h"
#include "task.h"
#include "delay.h"
#include "uart1.h"
#include "uart3.h"
#include "uart6.h"
#include "pwm.h"
#include "imu.h"
#include "gpio.h"
#include "includes.h"
//#include "uart8.h"

#include "SCServo.h" // 飞特串口舵机
#include "SMS_STS.h"

int continue_flag=0; // 不可以删除，淦



void Gun_task(void *p_arg)
{
	OS_ERR err;
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



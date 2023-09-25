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
#include "Judge_System.h"

#include "SCServo.h" // 飞特串口舵机
#include "SMS_STS.h"

int continue_flag=0;
int Pos;
int Speed;
int Load;
int Voltage;
int Temper;
int Move;
int Current;

//舵机控制，简单写
void STS3032_ServoControl(void)
{
	WritePosEx(1, 2000, 2250, 50);//舵机(ID1),以最高速度V=2250步/秒,加速度A=50(50*100步/秒^2),运行至P1=4095
//	// 第一个参数是ID号，默认为1
//	// 第二个参数是位置，一圈为4096
//	// 第三个参数是最高速度 （pulse/s）
//	// 第四个参数是加速度 （pulse/s^2）
}

void Gun_task(void *p_arg)
{
	OS_ERR err;
//	STS3032_ServoControl(); //STS3032舵机测试
	while(1)
	{  
//		STS3032_ServoControl(); //STS3032舵机测试

		
		if(FeedBack(1)!=-1){
			Pos = ReadPos(-1);
			Speed = ReadSpeed(-1);
			Load = ReadLoad(-1);
			Voltage = ReadVoltage(-1);
			Temper = ReadTemper(-1);
			Move = ReadMove(-1);
			Current = ReadCurrent(-1);
			//printf("Pos:%d\n", Pos);
			//printf("Speed:%d\n", Speed);
			//printf("Load:%d\n", Load);
			//printf("Voltage:%d\n", Voltage);
			//printf("Temper:%d\n", Temper);
			//printf("Move:%d\n", Move);
			//printf("Current:%d\n", Current);
			delay_us(10);
		}
		Pos = ReadPos(1);
		
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



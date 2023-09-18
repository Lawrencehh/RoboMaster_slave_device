//#include "can1.h"
//#include "task.h"
//#include "delay.h"
//#include "uart1.h"
//#include "uart2.h"
//#include "uart3.h"
//#include "pwm.h"
//#include "imu.h"
//#include "gpio.h"
//extern float pos_x;//三位坐标左右偏移
//extern float pos_y;//三维坐标上下便宜
//extern float pos_z;//三维坐标远近偏移
////////////u8 time_again_flag=1;    //串口2回调函数的标志位
//////////////回调函数标志位初值为1，如果执行一次回调函数就变回1
//////////////开始计时的时候变成0 计时结束就执行回调函数变成1
//extern u8 again_spin;    //重新旋转的信号
//extern u8 flag_get_pos;

//void Time_task(void *p_arg)
//{
//	OS_ERR err;
//	p_arg = p_arg;
//	
//while(1)
//{
//			USART_SendData(USART3,again_spin);//云台向底盘发送数据  0为底盘动 云台旋转 1为全部停止

//        if(pos_x==0.0f&&pos_y==0.0f&&pos_z==0.0f)
//				{	
//           again_spin=0;     //重新旋转的标志位 收到有效数据的标志位
//					 flag_get_pos=0;  //没有数据变0     数据整定好以后的标志位
//				}				

//		OSTimeDly(60,OS_OPT_TIME_PERIODIC,&err); 
//	}
//}

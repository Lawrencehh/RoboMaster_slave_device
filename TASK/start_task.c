#include "can1.h"
#include "can2.h"
#include "task.h"
#include "delay.h"
#include "uart1.h"
#include "uart3.h"
#include "usart69050.h"	
#include "pwm.h"
#include "imu.h"
#include "gpio.h"
#include "Judge_System.h"
#include "pstwo.h"
#include "includes.h"
 
//任务优先级
#define START_TASK_PRIO		3
//任务堆栈大小	
#define START_STK_SIZE 		128
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];

#define INIT_TASK_PRIO		4
#define INIT_STK_SIZE 		128
OS_TCB InitTaskTCB;
CPU_STK INIT_TASK_STK[INIT_STK_SIZE];

#if EN_CHASIS_TASK
#define CHASIS_TASK_PRIO		6
#define CHASIS_STK_SIZE 		512
OS_TCB CHASISTaskTCB;
CPU_STK CHASIS_TASK_STK[CHASIS_STK_SIZE];
#endif

#if EN_GIMBAL_TASK
#define GIMBAL_TASK_PRIO		5
#define GIMBAL_STK_SIZE 		512
OS_TCB GIMBALTaskTCB;
CPU_STK GIMBAL_TASK_STK[GIMBAL_STK_SIZE];
#endif


#if EN_GUN_TASK
#define GUN_TASK_PRIO		6
#define GUN_STK_SIZE 		128
OS_TCB GUNTaskTCB;
CPU_STK GUN_TASK_STK[GUN_STK_SIZE];
#endif

#if EN_TIME_TASK
#define TIME_TASK_PRIO		6
#define TIME_STK_SIZE 		128
OS_TCB TIMETaskTCB;
CPU_STK TIME_TASK_STK[TIME_STK_SIZE];
#endif

OS_TMR tmr1;    //定时器1
OS_TMR tmr2;    //定时器2

void tmr1_callback(void *p_tmr, void *p_arg); 	//定时器1回调函数
void tmr2_callback(void *p_tmr, void *p_arg); 	//定时器2回调函数

//开始任务函数
void start_task(void *p_arg)
	{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
		delay_ms(3000);
	
	//创建定时器1
	OSTmrCreate((OS_TMR		*)&tmr1,		//定时器1
                (CPU_CHAR	*)"tmr1",		//定时器名字
                (OS_TICK	 )50,			//50*10=500ms
                (OS_TICK	 )0,         
                (OS_OPT		 )OS_OPT_TMR_ONE_SHOT, //单次定时器
                (OS_TMR_CALLBACK_PTR)tmr1_callback,//定时器1回调函数
                (void	    *)0,			//参数为0
                (OS_ERR	    *)&err);		//返回的错误码					
  //创建定时器2
	OSTmrCreate((OS_TMR		*)&tmr2,		//定时器1
                (CPU_CHAR	*)"tmr2",		//定时器名字
                (OS_TICK	 )10,			//10*10=100ms
                (OS_TICK	 )0,         
                (OS_OPT		 )OS_OPT_TMR_ONE_SHOT, //单次定时器
                (OS_TMR_CALLBACK_PTR)tmr2_callback,//定时器1回调函数
                (void	    *)0,			//参数为0
                (OS_ERR	    *)&err);		//返回的错误码			
	OS_CRITICAL_ENTER();	//进入临界区					
//创建初始化任务
	OSTaskCreate(  (OS_TCB 	* )&InitTaskTCB,		
				         (CPU_CHAR	* )"Init task", 		
                 (OS_TASK_PTR )Init_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )INIT_TASK_PRIO,     	
                 (CPU_STK   * )&INIT_TASK_STK[0],	
                 (CPU_STK_SIZE)INIT_STK_SIZE/10,	
                 (CPU_STK_SIZE)INIT_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);		
									
#if EN_CHASIS_TASK
	OSTaskCreate(  (OS_TCB 	* )&CHASISTaskTCB,		
				         (CPU_CHAR	* )"Chasis task", 		
                 (OS_TASK_PTR )Chasis_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )CHASIS_TASK_PRIO,     	
                 (CPU_STK   * )&CHASIS_TASK_STK[0],	
                 (CPU_STK_SIZE)CHASIS_STK_SIZE/10,	
                 (CPU_STK_SIZE)CHASIS_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
#endif					
								 
#if EN_GIMBAL_TASK
	OSTaskCreate(  (OS_TCB 	* )&GIMBALTaskTCB,		
				         (CPU_CHAR	* )"Gimbal task", 		
                 (OS_TASK_PTR )Gimbal_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )GIMBAL_TASK_PRIO,     	
                 (CPU_STK   * )&GIMBAL_TASK_STK[0],	
                 (CPU_STK_SIZE)GIMBAL_STK_SIZE/10,	
                 (CPU_STK_SIZE)GIMBAL_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
#endif								 
										 
								 
#if EN_GUN_TASK
	OSTaskCreate(  (OS_TCB 	* )&GUNTaskTCB,		
				         (CPU_CHAR	* )"Gun task", 		
                 (OS_TASK_PTR )Gun_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )GUN_TASK_PRIO,     	
                 (CPU_STK   * )&GUN_TASK_STK[0],	
                 (CPU_STK_SIZE)GUN_STK_SIZE/10,	
                 (CPU_STK_SIZE)GUN_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
#endif								 
	
#if EN_TIME_TASK
	OSTaskCreate(  (OS_TCB 	* )&TIMETaskTCB,		
				         (CPU_CHAR	* )"Time task", 		
                 (OS_TASK_PTR )Time_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )TIME_TASK_PRIO,     	
                 (CPU_STK   * )&TIME_TASK_STK[0],	
                 (CPU_STK_SIZE)TIME_STK_SIZE/10,	
                 (CPU_STK_SIZE)TIME_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
#endif						
								 
//		OS_TaskSuspend((OS_TCB*)&CHASISTaskTCB,&err);       //挂起底盘任务								 
		//OS_TaskSuspend((OS_TCB*)&GIMBALTaskTCB,&err);       //挂起云台任务
//    OS_TaskSuspend((OS_TCB*)&GUNTaskTCB,&err);          //挂起射击任务
//	  OS_TaskSuspend((OS_TCB*)&TIMETaskTCB,&err);          //挂起时间任务
//	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		    //挂起开始任务			
								 
	OS_CRITICAL_EXIT();	//推出临界区   
}

void StartTaskCreate(void){
	OS_ERR err;
	OSTaskCreate(  (OS_TCB 	* )&StartTaskTCB,		//任务控制块
								 (CPU_CHAR * )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK  * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY)0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT    )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
}

void Init_task(void *p_arg){
		OS_ERR err;
		p_arg = p_arg;
	 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组配置
  	User_GPIO_Init();
//		TEL_USART1_Init();			  	//遥控配置
		MPU_Init1(); 
   	PS2_Init();
  	PS2_SetInit();
		CAN1_Init();                //CAN总线配置
	  CAN2_Init();                //CAN2总线配置
//		MPU6050_42mm_Gyro_calibration();
		TX2_USART3_Init();       //TX2通信
		
	  TIM2_PWM_Init();
	  TIM4_PWM_Init();
	  TIM5_PWM_Init();
	  TIM8_PWM_Init();
//		//Heat_PWM_Init();
//	  USART6_init();     //陀螺仪的串口
  	
	

//		OS_TaskResume((OS_TCB*)&CHASISTaskTCB,&err);
	  OS_TaskResume((OS_TCB*)&GIMBALTaskTCB,&err);     //云台
//		OS_TaskResume((OS_TCB*)&GUNTaskTCB,&err);
//		OS_TaskResume((OS_TCB*)&TIMETaskTCB,&err);
// 
	OS_TaskSuspend((OS_TCB*)&InitTaskTCB,&err);		   //		
}


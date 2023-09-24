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
#include <stdint.h>  // 包含int16_t的头文件

int32_t GM6020_setpoint = 20000;  // 设定点
int32_t GM6020_output_limit = 30000; // 对输出电压作出限制

int32_t GM6020_rotation_count = 0;  // 旋转计数（每完成一圈增加1或减少1）
int32_t GM6020_absolute_position = 0;  // 绝对位置
int32_t error = 0; // 误差值
int32_t GM6020_position_difference = 0; // 计算位置差

// PID参数
double GM6020_Kp = 10;  // 比例常数
double GM6020_Ki = 0.001;  // 积分常数
double GM6020_Kd = 0.001; // 微分常数
int32_t GM6020_output = 0;    // 输出电压 -30000 - 30000 防止溢出
int32_t prev_error = 0; // 上一次的误差
int32_t integral = 0;   // 误差积分
int32_t derivative; 	// 误差微分			

// 更新GM6020_PID控制器
void GM6020_update_pid() {
	
		// 计算位置差
		GM6020_position_difference = GM6020_current_raw_position - GM6020_last_raw_position;
		// 检查是否有位置包裹
		if (abs(GM6020_position_difference) > 4096) {  // 8191 / 2 = 4095.5
				if (GM6020_position_difference > 0) {
						--GM6020_rotation_count;
				} else {
						++GM6020_rotation_count;
				}
		}
		// 计算绝对位置
		GM6020_absolute_position = GM6020_current_raw_position + GM6020_rotation_count * 8192;

		
    error = GM6020_setpoint - GM6020_absolute_position;  // 计算误差
    integral += error;                // 计算误差积分
    derivative = error - prev_error;  // 计算误差微分
    // 计算输出电压
		GM6020_output = GM6020_Kp * error + GM6020_Ki * integral + GM6020_Kd * derivative; // PID：当系统有持续的偏差（steady-state error）时，使用PID。
	
		// 限制GM6020_output在-GM6020_output_limit到GM6020_output_limit之间
    if (GM6020_output > GM6020_output_limit) {
        GM6020_output = GM6020_output_limit;
    } else if (GM6020_output < -GM6020_output_limit) {
        GM6020_output = -GM6020_output_limit;
    }
		
		// 发送控制指令
		Can_Send_Msg(GM6020_output,0,0,0);
    prev_error = error;  // 更新上一次的误差
		// 更新上一次的原始位置
		GM6020_last_raw_position = GM6020_current_raw_position;
		// 更新当前位置
		GM6020_current_raw_position = GripperMotor_205_t.position; 
}

void Gimbal_task(void *p_arg)
{
	OS_ERR err;


	while(1)
	{  	

		
			GM6020_update_pid();  // 更新GM6020_PID控制器
			OSTimeDly(10,OS_OPT_TIME_PERIODIC,&err); //延时10ms
  } 
}




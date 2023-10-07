#include "can1.h"
#include "can2.h"
#include "task.h"
#include "delay.h"
#include "pstwo.h"
#include "uart3.h"
#include "uart6.h"
//#include "uart8.h"
#include "pwm.h"
#include "imu.h"
#include "gpio.h"
#include "includes.h"
#include "math.h"
#include <stdint.h>  // 包含int16_t的头文件

#include "SCServo.h" // 飞特串口舵机
#include "SMS_STS.h"


void USART6_Init(void);
/****************************************************************************
													GM6020电机的PID代码
****************************************************************************/

int32_t GM6020_output_limit = 30000; // 对输出电压作出限制
int32_t GM6020_rotation_count = 0;  // 旋转计数（每完成一圈增加1或减少1）
int32_t GM6020_absolute_position = 0;  // 绝对位置
int32_t GM6020_error = 0; // 误差值
int32_t GM6020_position_difference = 0; // 计算位置差
// PID参数
double GM6020_Kp = 100;  // 比例常数
double GM6020_Ki = 0;  // 积分常数
double GM6020_Kd = 10; // 微分常数
int32_t GM6020_output = 0;    // 输出电压 -30000 - 30000 防止溢出
int32_t GM6020_prev_error = 0; // 上一次的误差
int32_t GM6020_integral = 0;   // 误差积分
int32_t GM6020_derivative; 	// 误差微分			
// 更新GM6020_PID控制器
void GM6020_update_pid() {
	

		// 计算位置差
		GM6020_position_difference = GM6020_current_raw_position - GM6020_last_raw_position;
		// 检查是否有位置包裹
		if (abs(GM6020_position_difference) > 180) {  
				if (GM6020_position_difference > 0) {
						--GM6020_rotation_count;
				} else {
						++GM6020_rotation_count;
				}
		}
		// 计算绝对位置
		if(reset_control){
			gripper_gm6020_position_control=0;
		}
		GM6020_absolute_position = GM6020_current_raw_position + GM6020_rotation_count * 360 - gripper_gm6020_position_reset_offset;
    GM6020_error = gripper_gm6020_position_control - GM6020_absolute_position;  // 计算误差
    GM6020_integral += GM6020_error;                // 计算误差积分
    GM6020_derivative = GM6020_error - GM6020_prev_error;  // 计算误差微分
    // 计算输出电压
		GM6020_output = GM6020_Kp * GM6020_error + GM6020_Ki * GM6020_integral + GM6020_Kd * GM6020_derivative; // PID：当系统有持续的偏差（steady-state error）时，使用PID。
		// 限制GM6020_output在-GM6020_output_limit到GM6020_output_limit之间
    if (GM6020_output > GM6020_output_limit) {
        GM6020_output = GM6020_output_limit;
    } else if (GM6020_output < -GM6020_output_limit) {
        GM6020_output = -GM6020_output_limit;
    }
		GM6020_output = PID_GM6020_Velocity(GM6020_output,GripperMotor_205_t.velocity); // pid for velocity
		// 发送控制指令
		if(reset_control == 0){
			GM6020_Can_Send_Msg(GM6020_output,0,0,0);
		}
    GM6020_prev_error = GM6020_error;  // 更新上一次的误差
		// 更新上一次的原始位置
		GM6020_last_raw_position = GM6020_current_raw_position;
		// 更新当前位置
		GM6020_current_raw_position = GripperMotor_205_t.position; 

}

int32_t PID_GM6020_Velocity(int32_t target_velocity,int32_t current_velocity) 
{
	const float Kp =10;  //16.5
	const float Ki=0;   //0.03
	const float Kd =0.0f;
    
	static float error_v[2] = {0.0,0.0};
	static float error_sum=0;
	static float error =0; 
	
	error_v[0] = error_v[1];
	error_v[1] = target_velocity - current_velocity;	
	error_sum += error_v[1];

	if(error_sum > 10000)  error_sum =  10000;      //
	else if(error_sum < -10000) error_sum = -10000;
	error = error_v[1]  * Kp 
				 +  error_sum * Ki 
				 + (error_v[1] - error_v[0]) * Kd; 
	
	if(error > GM6020_output_limit)  error = GM6020_output_limit;
	else if(error < -GM6020_output_limit) error = -GM6020_output_limit;
	return error;
}

/****************************************************************************/

/****************************************************************************
													C610电机的PID代码
****************************************************************************/
int32_t C610_output_limit = 10000; // 对输出电流作出限制
int32_t C610_rotation_count = 0;  // 旋转计数（每完成一圈增加1或减少1）
int32_t C610_absolute_position = 0;  // 绝对位置
int32_t C610_error = 0; // 误差值
int32_t C610_position_difference = 0; // 计算位置差
// PID参数
double C610_Kp = 1;  // 比例常数   需要整定
double C610_Ki = 0;  // 积分常数
double C610_Kd = 10; // 微分常数
int32_t C610_output = 0;    // 输出电压 -10000 - 10000 防止溢出
int32_t C610_prev_error = 0; // 上一次的误差
int32_t C610_integral = 0;   // 误差积分
int32_t C610_derivative; 	// 误差微分	
// 更新C610_PID控制器
void C610_update_pid() 
	{
		// 计算位置差
		C610_position_difference = C610_current_raw_position - C610_last_raw_position;
		// 检查是否有位置包裹
		if (abs(C610_position_difference) > 180) {  
				if (C610_position_difference > 0) {
						--C610_rotation_count;
				} else {
						++C610_rotation_count;
				}
		}
		// 计算绝对位置
		if(reset_control){
			gripper_c610_position_control=0;
		}
		C610_absolute_position = C610_current_raw_position + C610_rotation_count * 360 - gripper_c610_position_reset_offset;
    C610_error = gripper_c610_position_control - C610_absolute_position;  // 计算误差
    C610_integral += C610_error;                // 计算误差积分
    C610_derivative = C610_error - C610_prev_error;  // 计算误差微分
    // 计算输出电压
		C610_output = 1 * (C610_Kp * C610_error + C610_Ki * C610_integral + C610_Kd * C610_derivative); // PID：当系统有持续的偏差（steady-state error）时，使用PID。
		// 限制C610_output在-C610_output_limit到C610_output_limit之间
    if (C610_output > C610_output_limit) {
        C610_output = C610_output_limit;
    } else if (C610_output < -C610_output_limit) {
        C610_output = -C610_output_limit;
    }
		C610_output = PID_C610_Velocity(C610_output,GripperMotor_201_t.velocity); // pid for velocity
		// 发送控制指令
		if(reset_control == 0){
		C610_Can_Send_Msg(C610_output,0,0,0);
		}
    C610_prev_error = C610_error;  // 更新上一次的误差
		// 更新上一次的原始位置
		C610_last_raw_position = C610_current_raw_position;
		// 更新当前位置
		C610_current_raw_position = GripperMotor_201_t.position; 
		
}
	
int32_t PID_C610_Velocity(int32_t target_velocity,int32_t current_velocity) 
{
	const float Kp =1;  //16.5
	const float Ki=0.001f;   //0.03
	const float Kd =0.0f;
    
	static float error_v[2] = {0.0,0.0};
	static float error_sum=0;
	static float error =0; 
	
	error_v[0] = error_v[1];
	error_v[1] = target_velocity - current_velocity;	
	error_sum += error_v[1];

	if(error_sum > 1000)  error_sum =  1000;      //
	else if(error_sum < -1000) error_sum = -1000;
	error = error_v[1]  * Kp 
				 +  error_sum * Ki 
				 + (error_v[1] - error_v[0]) * Kd; 
	
	if(error > 20000)  error = 20000;
	else if(error < -20000) error = -20000;
	return error;
}

/****************************************************************************/

/****************************************************************************
															STS3032舵机控制
****************************************************************************/

void STS3032_ServoControl(void){
	
	// 仅当值改变的时候执行指令
	if(gripper_sts3032_position_control != last_sts3032_control_value){
		int16_t position_control = 0;
		
		position_control = (gripper_sts3032_position_control+gripper_sts3032_position_reset_offset)*4096/360;
		position_control = (position_control % 4096 + 4096) % 4096; // 确保为0-4095的正整数
		if(reset_control){
			position_control=0;
		}
		if(reset_control != 1){
			WritePosEx(1, position_control, 2250, 50);//舵机(ID1),以最高速度V=2250步/秒,加速度A=50(50*100步/秒^2),运行至P1=4095
			// 第一个参数是ID号，默认为1
			// 第二个参数是位置，一圈为4096
			// 第三个参数是最高速度 （pulse/s）
			// 第四个参数是加速度 （pulse/s^2）
		}
	}
}
/****************************************************************************/

void Gimbal_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;

	STS3032_ServoControl();
	while(1)
	{  	
	
			
			GM6020_update_pid(); // 更新GM6020_PID控制器
			C610_update_pid();  // 更新C610_PID控制器
			STS3032_ServoControl();

			OSTimeDly(2,OS_OPT_TIME_PERIODIC,&err); //延时2ms
  } 
}




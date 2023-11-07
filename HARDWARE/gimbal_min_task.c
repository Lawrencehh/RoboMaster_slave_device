
#include "includes.h"
#include "can2.h"
#include "task.h"
#include "uart1.h"
#include "uart3.h"
#include "math.h"


float target_angle_Yaw2=270.0f,target_angle_Pitch2=150.0f;//小枪管自瞄
int16_t output_207=0,output_209=0;
float this_angle_Pitch_min,this_angle_Yaw_min;
u8 flag_Vshift=0;
double Camera_Inc_Pitch_min,Camera_Inc_Yaw_min,X_distance,Y_distance,Z_distance,distance_min;
static float inc_205_min[2]={0.0f,0.0f},error205_v_min[2]={0.0f,0.0f};
static float inc_206_min[2]={0.0f,0.0f},error206_v_min[2]={0.0f,0.0f};
double_t limit_k_min,limit_kk_min;
int16_t count_n1_min=0,flag_count_start_min=0;
float Output_Camera_Inc_Yaw_min=0,Output_Camera_Inc_Pitch_min=0,yaw_mid=0.0f;	



void Gimbal_min_task(void *p_arg)    //原本为小云台的控制，后拆除，已不用
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{

		
		OSTimeDly(4,OS_OPT_TIME_PERIODIC,&err);
	}
}



int16_t PID_207_Velocity(int16_t target_velo,int16_t current_velo)  //yaw轴速度环
{
	const float Kp =400.0f;  //27
	const float Ki=0.15f;   //0.05
	const float Kd =0.0f;
    
	static int32_t error_v[2] = {0.0,0.0};
	static int32_t error_sum=0;
	static int32_t error =0;
			
	error_v[0] = error_v[1];
	error_v[1] = -target_velo+current_velo;	
	error_sum += error_v[1];

	if(error_sum > 20000)  error_sum =  20000;      //抑制积分饱和
	else if(error_sum < -20000) error_sum = -20000;
	error = error_v[1]  * Kp
				 +  error_sum * Ki 
				 + (error_v[1] - error_v[0]) * Kd;  
	
	if(error > 28000)  error = 28000;
	else if(error < -28000) error = -28000;
	return error;
}

int16_t PID_207_Position(float target_pos,float current_pos)  //yaw轴位置环
{
	const float Kp =16.0f;   //15
	const float Ki=0.0f;
	const float Kd =0.0f;
    
	static float error_v[2] = {0.0,0.0};
	static float error_sum=0;
	static float error =0;
	
//	if(target_pos<=105)target_pos=105;
//	if(target_pos>=150)target_pos=150;
			
	error_v[0] = error_v[1];
	error_v[1] = -target_pos+current_pos;	
	error_sum += error_v[1];

	if(error_sum > 8000)  error_sum =  8000;      //抑制积分饱和
	else if(error_sum < -8000) error_sum = -8000;
	error = error_v[1]  * Kp
				 +  error_sum * Ki 
				 + (error_v[1] - error_v[0]) * Kd; 
	
	if(error > 20000)  error = 20000;
	else if(error < -20000) error = -20000;
	return error;
}



int16_t PID_209_Velocity(int16_t target_velo,int16_t current_velo)  //Pitch轴速度环
{
	const float Kp =27.0f;  //27
	const float Ki=0.05f;   //0.15
	const float Kd =0.0f;
    
	static int32_t error_v[2] = {0.0,0.0};
	static int32_t error_sum=0;
	static int32_t error =0;
			
	error_v[0] = error_v[1];
	error_v[1] = target_velo-current_velo;	
	error_sum += error_v[1];

	if(error_sum > 8000)  error_sum =  8000;      //抑制积分饱和
	else if(error_sum < -8000) error_sum = -8000;
	error = error_v[1]  * Kp
				 +  error_sum * Ki 
				 + (error_v[1] - error_v[0]) * Kd; 
	
	if(error > 5000)  error = 5000;
	else if(error < -5000) error = -5000;
	return error;
}

int16_t PID_209_Position(float target_pos,float current_pos)  //Pitch轴位置环
{
	const float Kp =25.0f;   //15
	const float Ki=0.0f;
	const float Kd =0.0f;
    
	static float error_v[2] = {0.0,0.0};
	static float error_sum=0;
	static float error =0;
//	
//	if(target_pos<=105)target_pos=105;
//	if(target_pos>=150)target_pos=150;
			
	error_v[0] = error_v[1];
	error_v[1] = -target_pos+current_pos;	
	error_sum += error_v[1];

	if(error_sum > 8000)  error_sum =  8000;      //抑制积分饱和
	else if(error_sum < -8000) error_sum = -8000;
	error = error_v[1]  * Kp
				 +  error_sum * Ki 
				 + (error_v[1] - error_v[0]) * Kd; 
	
	if(error > 8000)  error = 8000;
	else if(error < -8000) error = -8000;
	return error;
}




#define FPS 200          //摄像头帧率
#define T_Finish  0.002f  //云台收敛时间
float Camera_PID_205_Position_min(float new_inc)
{
	static float error =0;
		
	/*更新数组*/	
	inc_205_min[1]=new_inc;
	error205_v_min[0]=error205_v_min[1];
	error205_v_min[1]=inc_205_min[1]-inc_205_min[0];
	inc_205_min[0]=inc_205_min[1];
	 
	/*线性拟合*/
	error = inc_205_min[1]																			 //零阶
					+ error205_v_min[1]*T_Finish*FPS;             									 //一阶
					+ (error205_v_min[1] - error205_v_min[0])*0.5f*T_Finish*T_Finish*FPS;    //二阶
	
	return error;
}

float Camera_PID_206_Position_min(float new_inc)
{
	static float error =0;
	
	/*更新数组*/	
	inc_206_min[1]=new_inc;
	error206_v_min[0]=error206_v_min[1];
	error206_v_min[1]=inc_206_min[1]-inc_206_min[0];
	inc_206_min[0]=inc_206_min[1];
	
	/*线性拟合*/
	error = inc_206_min[1] 																							 //零阶
					+ error206_v_min[1]*T_Finish*FPS;             									 //一阶
					+ (error206_v_min[1] - error206_v_min[0])*0.5f*T_Finish*T_Finish*FPS;    //二阶
	
	return error;
}
float PID_camera_yaw_min(float target_pos,float current_pos)  
{
	float k; 
	k=(1-fabs(target_pos-current_pos)/30);
	const float Kp =0.01;          //0.35
	const float Ki=0.0f;          //0.001
	const float Kd =0.0f;          //       突然出现防止超调
    
	static float error_v[2] = {0.0,0.0};
	static float error_sum=0;
	static float error =0;
			
	error_v[0] = error_v[1];
	error_v[1] = -target_pos+current_pos;	
	error_sum += error_v[1];

	if(error_sum > 2000)  error_sum =  2000;      //抑制积分饱和
	else if(error_sum < -2000) error_sum = -2000;
	error = error_v[1]  * Kp *k
				 +  error_sum * Ki 
				 + (error_v[1] - error_v[0]) * Kd; 
	
	if(error > 5000)  error = 5000;
	else if(error < -5000) error = -5000;
	return error;
}

float PID_camera_pitch_min(float target_pos,float current_pos)  
{
	float k;
	k=(1-fabs(target_pos-current_pos)/30);
	const float Kp =0.08;          //18.0f;
	const float Ki=0.0f;
	const float Kd =0.0f;
    
	static float error_v[2] = {0.0,0.0};
	static float error_sum=0;
	static float error =0;
			
	error_v[0] = error_v[1];
	error_v[1] = -target_pos+current_pos;	
	error_sum += error_v[1];

	if(error_sum > 2000)  error_sum =  2000;      //抑制积分饱和
	else if(error_sum < -2000) error_sum = -2000;
	error = error_v[1]  * Kp * k
				 +  error_sum * Ki 
				 + (error_v[1] - error_v[0]) * Kd; 
	
	if(error > 5000)  error = 5000;
	else if(error < -5000) error = -5000;
	return error;
}







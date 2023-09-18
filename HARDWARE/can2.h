#ifndef __CAN2_H__
#define __CAN2_H__


#include <stm32f4xx.h>


void CAN2_Init(void);
void Chasis_ESC_Send_Can2(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204);
void Gimbal_ESC_Send_Can2(int16_t current_205,int16_t current_206,int16_t current_207,int16_t current_208);
void Gimbal_ESC_Send_Can2_yaw(int16_t current_205,int16_t current_206,int16_t current_207,int16_t current_208);
typedef struct
{
	int16_t phrase;
	int16_t velocity;
	int16_t elecrent;
	int8_t  temper;
}Chasis_ID_t_Can2;


typedef struct
{
	int16_t q0;
	int16_t q1;
	int16_t q2;
	int16_t q3;
}Quaternion_t;                //四元数
 

typedef struct
{
	float yaw;
	float pitch;
	float roll;
}Euler_angle_t;               //欧拉角       


typedef struct
{
	int16_t imu_wx;
	int16_t imu_wy;
	int16_t imu_wz;
	int16_t temper;
}angular_velocity_t;           //角速度


typedef struct
{
	int16_t imu_ax;
	int16_t imu_ay;
	int16_t imu_az;
	int16_t time;
}acceleration_t;               //加速度


typedef struct
{
	int16_t X_axis;
	int16_t Y_axis;
	int16_t Z_axis;
}magnetic_field_t;            //磁场





extern Chasis_ID_t_Can2 Firction_201_t,Firction_202_t,Bodan_203_t,Bodan_204_t;
extern int32_t phase_bodan;
extern float anger_bodan,anger_bodan_2006,anger_bodan_min;
extern int32_t phase_bodan,phase[2],phase_mid;
extern int16_t round_bodan;
extern Chasis_ID_t_Can2 Gimbal_205_t_Can2,Gimbal_206_t_Can2,Gimbal_208_t_Can2;
extern float anger_pitch1,anger_yaw1;
extern Quaternion_t Quaternion;
extern Euler_angle_t Euler_angle;
extern angular_velocity_t angular_velocity;
extern acceleration_t acceleration;
extern magnetic_field_t magnetic_field;


#endif

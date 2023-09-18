#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mpu6050_interrupt.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "myiic.h"
#include "mpu6050.h"
#include "sys.h"
#include "delay.h"
//extern float avemousex,avemousey;
//extern float wz;
//float pitch,roll,yaw; 		//欧拉角
//short aacx,aacy,aacz;		//加速度传感器原始数据
//short gyrox,gyroy,gyroz;	//陀螺仪原始数据	 
//float Gyro_X,Gyro_Y,Gyro_Z;

//extern int DD;
//extern char shooting_flag;
//extern int shootx,flag_1000ms;
//int shoot_flag;
//int gyroADC_X_offset=0,gyroADC_Y_offset=0,gyroADC_Z_offset=0,yaw_offset=0;
//extern float posx,posy,posz,pos_x,pos_y,pos_z;

//int auto_m;
//extern int zimiaoo;

//float pid_yaw,pid_pitch;

//#define MAXANGLE		101
//#define MINANGLE    60
//#define YAW_MAX     459
//#define YAW_MINI    277

//void MPU6050_ReadData(void)
//{
//    mpu_dmp_get_data(&pitch,&roll,&yaw);
//		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
//    Gyro_X = (float)(gyrox - gyroADC_X_offset)/65.5f;     //见datasheet 32 of 47
//    Gyro_Y = (float)(gyroy - gyroADC_Y_offset)/65.5f;     //见datasheet 32 of 47
//    Gyro_Z = (float)(gyroz - gyroADC_Z_offset)/65.5f;     //见datasheet 32 of 47    
//}
//extern void IIC_Delay(unsigned int t);
//void MPU6050_Gyro_calibration(void)
//{
//	u16 i;
//	float x_temp=0,y_temp=0,z_temp=0;
//	
//	for(i=0; i<1000; i++)
//	{
//		MPU6050_ReadData();	
//		IIC_Delay(10);
//		x_temp=x_temp+gyrox;
//		y_temp=y_temp+gyroy;
//		z_temp=z_temp+gyroz;
//	}			
//	
//	x_temp=x_temp/1000;
//	y_temp=y_temp/1000;	
//	z_temp=z_temp/1000;

//	gyroADC_X_offset=(int)x_temp;
//	gyroADC_Y_offset=(int)y_temp;
//	gyroADC_Z_offset=(int)z_temp;
//}

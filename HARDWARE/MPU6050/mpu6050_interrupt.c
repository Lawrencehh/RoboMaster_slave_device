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

//extern int16_t encoder_205;
//extern int16_t encoder_206;
//extern float pitchy,mousey;
//extern float yawx,mousex[2];
//extern char right_flag,left_flag;
//extern int mode_flag;
//extern int auto_aim_flag;
//extern int auto_mode_flag;
//extern uint16_t keyboard;
//float target_pitch_angle=76;     // 115-165目标PITCH轴
//float target_yaw_angle=364;         //277-459 ，yaw轴目标角度范围
//float this_206_angle = 0.0;       //本次pitch轴的角度
//float velo_206_output= 0.0;       //pitch轴速度环函数输出值
//float pos_206_output = 0.0;       //pitch轴位置环函数输出值
//float this_205_angle = 0.0;       //本次yaw轴的角度
//float velo_205_output= 0.0;       //yaw轴速度环函数输出值
//float pos_205_output = 0.0;  
//float pitch_position=76;
//float yaw_position=364;
//extern char slow_flag;
//extern float Angle_Z;
//static int single=1,follow=1,spin=1;
extern float avemousex,avemousey;
extern float wz;
float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据	 
float Gyro_X,Gyro_Y,Gyro_Z;

extern int DD;
extern char shooting_flag;
extern int shootx,flag_1000ms;
int shoot_flag;
int gyroADC_X_offset=0,gyroADC_Y_offset=0,gyroADC_Z_offset=0,yaw_offset=0;
extern float posx,posy,posz,pos_x,pos_y,pos_z;

int auto_m;
extern int zimiaoo;

float pid_yaw,pid_pitch;

#define MAXANGLE		101
#define MINANGLE    60
#define YAW_MAX     459
#define YAW_MINI    277

void delay_imu_ms(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=42000;
		while(a--);
	}
}
void MPU6050_ReadData(void)
{
    mpu_dmp_get_data(&pitch,&roll,&yaw);
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
    Gyro_X = (float)(gyrox - gyroADC_X_offset)/65.5f;     //见datasheet 32 of 47
    Gyro_Y = (float)(gyroy - gyroADC_Y_offset)/65.5f;     //见datasheet 32 of 47
    Gyro_Z = (float)(gyroz - gyroADC_Z_offset)/65.5f;     //见datasheet 32 of 47    
}

void MPU6050_Gyro_calibration(void)
{
	u16 i;
	float x_temp=0,y_temp=0,z_temp=0;
	
	for(i=0; i<1000; i++)
	{
		MPU6050_ReadData();
		delay_imu_ms(3);
	
		x_temp=x_temp+gyrox;
		y_temp=y_temp+gyroy;
		z_temp=z_temp+gyroz;
	}			
	
	x_temp=x_temp/1000;
	y_temp=y_temp/1000;	
	z_temp=z_temp/1000;

	gyroADC_X_offset=(int)x_temp;
	gyroADC_Y_offset=(int)y_temp;
	gyroADC_Z_offset=(int)z_temp;
	
}

//void yuntai(void)
//	{  
//		
////		if(mode_flag!=11)
////		{
////		if(slow_flag==0)
////		{
////				target_yaw_angle-=(3.0f*0.007f*avemousex);//+0.064f/7*wz
////			if(right_flag==1||mode_flag==1){if(avemousex>-6&&avemousex<6)avemousex=0;}
////		}
////		else
////			target_yaw_angle-=(1.5f*0.007f*avemousex);

////			if(slow_flag==0)
////				target_pitch_angle-=(3.0f*0.007f*avemousey);//+0.064f/7);
////			else
////				target_pitch_angle-=(1.5f*0.007f*avemousey);//+0.064f/7);
////		}
//			
//if(mode_flag!=10)
//{
//				if(pitchy>20)
//				target_pitch_angle+=1.1f;
//				if(pitchy<-20)
//				target_pitch_angle-=1.1f;	
//				if(yawx>5)
//				{	target_yaw_angle-=1.25f;}
//				if(yawx<-5)
//				{target_yaw_angle+=1.25f;	}
//}

//			this_206_angle =encoder_206* 0.044f;        //机械角度;cyq:转化为欧拉角
//			this_205_angle =encoder_205* 0.044f;        //机械角度;cyq:转化为欧拉角；77~255
//			if (target_pitch_angle>MAXANGLE)target_pitch_angle = MAXANGLE;
//			if (target_pitch_angle<MINANGLE)target_pitch_angle = MINANGLE;
//				if(mode_flag!=10)
//				{
//			pos_206_output = Pos_Con_206(this_206_angle,target_pitch_angle);
//			velo_206_output =Velo_Con_206(Gyro_Y,pos_206_output);
//				}
//				
//	  if(right_flag==1||mode_flag==1||mode_flag==4)                          //单轴模式
//	  {
//			
//		
//		spin=1;
//		follow=1;
//		target_yaw_angle=364;

//		if (target_yaw_angle>YAW_MAX)	target_yaw_angle = YAW_MAX;
//		if (target_yaw_angle<YAW_MINI)	target_yaw_angle = YAW_MINI;
//		pos_205_output = Pos_Con_205(this_205_angle,target_yaw_angle);
//		velo_205_output =Velo_Con_205(Gyro_Z,pos_205_output);
//	}
//		if(mode_flag==10)                                                       //定位打击模式
//		{
//			if(DD==1)
//			{
//			pitch_position=this_206_angle;
//			yaw_position=this_205_angle;
//				DD=0;
//			}
//		if (pitch_position>MAXANGLE)	pitch_position= MAXANGLE;
//		if (pitch_position<MINANGLE)	pitch_position= MINANGLE;
//		if 	(yaw_position>YAW_MAX)			yaw_position = YAW_MAX;
//		if 	(yaw_position<YAW_MINI)			yaw_position = YAW_MINI;
//		spin=1;
//		follow=1;
//		pos_206_output	=Pos_Con_206(this_206_angle,pitch_position);
//		velo_206_output =Velo_Con_206(Gyro_Y,pos_206_output);

//		pos_205_output = Pos_Con_205(this_205_angle,yaw_position);
//		velo_205_output =Velo_Con_205(Gyro_Z,pos_205_output);
//		}
//    
//	if(mode_flag==2)                          //扭腰模式
//	{
//			
//		if(this_205_angle>YAW_MAX||this_205_angle<YAW_MINI)	
//		{
//			target_yaw_angle=yaw;
//		}

//		if(target_yaw_angle-yaw<-300) //Jump by counterclockwise
//		{
//			target_yaw_angle+=360;
//		}
//		if(target_yaw_angle-yaw>300)  //Jump by clockwise
//		{
//			target_yaw_angle-=360;
//		}
//			
//		single=1;
//		follow=1;
//			
//		if(spin==1)
//		{
//			target_yaw_angle=yaw;
//			spin=0;
//		}
//		pos_205_output = Pos_Con_205(yaw,target_yaw_angle);
//		velo_205_output =Velo_Con_205(Gyro_Z,pos_205_output);

//	}
//		if(mode_flag==9)                          //扭腰模式
//	{
//			
//		if(this_205_angle>YAW_MAX||this_205_angle<YAW_MINI)	
//		{
//			target_yaw_angle=yaw;
//		}

//		if(target_yaw_angle-yaw<-300) //Jump by counterclockwise
//		{
//			target_yaw_angle+=360;
//		}
//		if(target_yaw_angle-yaw>300)  //Jump by clockwise
//		{
//			target_yaw_angle-=360;
//		}
//			
//		single=1;
//		follow=1;
//			
//		if(spin==1)
//		{
//			target_yaw_angle=yaw-45;
//			spin=0;
//		}
//		pos_205_output = Pos_Con_205_spin(yaw,target_yaw_angle);
//		velo_205_output =Velo_Con_205_spin(Gyro_Z,pos_205_output);

//	}
//	if(mode_flag==8)                          //扭腰模式
//	{
//			
//		if(this_205_angle>YAW_MAX||this_205_angle<YAW_MINI)	
//		{
//			target_yaw_angle=yaw;
//		}

//		if(target_yaw_angle-yaw<-300) //Jump by counterclockwise
//		{
//			target_yaw_angle+=360;
//		}
//		if(target_yaw_angle-yaw>300)  //Jump by clockwise
//		{
//			target_yaw_angle-=360;
//		}
//			
//		single=1;
//		follow=1;
//			
//		if(spin==1)
//		{
//			target_yaw_angle=yaw+45;
//			spin=0;
//		}
//		pos_205_output = Pos_Con_205_spin(yaw,target_yaw_angle);
//		velo_205_output =Velo_Con_205_spin(Gyro_Z,pos_205_output);

//	}
//	
//       if(zimiaoo==0&&(right_flag==3||mode_flag==3||mode_flag==6||mode_flag==7))                          //跟随模式
//	{	
//		
//					
//									
//		if(this_205_angle>YAW_MAX||this_205_angle<YAW_MINI)	
//		{
//			target_yaw_angle=yaw;
//		}

//		if(target_yaw_angle-yaw<-300) //消除yaw值跳变影响
//		{
//			target_yaw_angle+=360;
//		}
//		if(target_yaw_angle-yaw>300) 
//		{
//			target_yaw_angle-=360;
//		}

//		spin=1;
//		single=1;
//		if(follow==1)
//		{
//			target_yaw_angle=yaw;
//			follow=0;
//		}
//		pos_205_output =Pos_Con_205_follow(yaw,target_yaw_angle);
//		velo_205_output =Velo_Con_205_follow(Gyro_Z,pos_205_output);
//	}
//	
//	
//	 if(zimiaoo==1&&(right_flag==3||mode_flag==3||mode_flag==6||mode_flag==7))                          //跟随模式
//		{
//			if(pos_x==0&&pos_y==0&&pos_z==0)
//					{
//						posx=0;posy=0;
//					}
//			else
//				
//				{
//				//pos_y-=50;//相机和枪管偏移
//				
//				posx=(float)(atan(pos_x/pos_z)*180/3.141592654);
//				posy=(float)((atan(pos_y/pos_z)*180/3.141592654))  ; 
//				/**************徐dalao写的子弹重力补偿*******************/
////									real_dis=sqrt(pos_y*pos_y+pos_z*pos_z);
////									temp1=Accel_Z*9.8f*real_dis*0.001f;
////									temp2=sqrt(temp1*temp1);
////									angle_g=asin(temp2/temp1)*57.29578f/4;//重力补偿角度
////									target_pitch_angle = target_pitch_angle+(abs(angle_g))*0.4;
//				/**************增量PID！！！！*******************/
//				
//				pid_pitch = SXT_pitch(posy,0); 
//				target_pitch_angle+=pid_pitch;//调整目标值可以实现枪管的固定偏移
//				
//				pid_yaw=SXT_yaw(posx,0);
//				target_yaw_angle-=pid_yaw;
//				
//				}
//					
//			
//			
//		if(this_205_angle>YAW_MAX||this_205_angle<YAW_MINI)	
//		{
//			target_yaw_angle=yaw;
//		}

//		if(target_yaw_angle-yaw<-300) //消除yaw值跳变影响
//		{
//			target_yaw_angle+=360;
//		}
//		if(target_yaw_angle-yaw>300) 
//		{
//			target_yaw_angle-=360;
//		}

//		spin=1;
//		single=1;
//		if(follow==1)
//		{
//			target_yaw_angle=yaw;
//			follow=0;
//		}
//		pos_205_output =Pos_Con_205_follow(yaw,target_yaw_angle);
//		velo_205_output =Velo_Con_205_follow(Gyro_Z,pos_205_output);
//	}
//	

//CAN1_SetMsg_yuntai(velo_206_output,velo_205_output);//picth负电流往上，正电流往下;yaw正电流往右//正电流往上，正电流往左
//}		
		

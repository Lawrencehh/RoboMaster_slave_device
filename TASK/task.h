#ifndef __TASK_H
#define __TASK_H

#include "stm32f4xx.h"
#include "sys.h"
#include "includes.h"



void StartTaskCreate(void);
void start_task(void *p_arg);
void Init_task(void *p_arg);

extern OS_TMR tmr1;  

/*************Chasis task*************/
#define EN_CHASIS_TASK 1u
void goahead(void);
void off(void);
void goback(void);
void turnleft(void);
void turnright(void);
void one_angle(void);
void two_angle(void);
void three_angle(void);
void four_angle(void);
void put_angle(void);
void pull_angle(void);
void open(void);
void close(void);


void Chasis_task(void *p_arg);
int16_t PID_201_Velocity(int16_t target_velo,int16_t current_velo);
int16_t PID_202_Velocity(int16_t target_velo,int16_t current_velo);
int16_t PID_203_Velocity(int16_t target_velo,int16_t current_velo);
int16_t PID_204_Velocity(int16_t target_velo,int16_t current_velo);
int16_t PID_chasis_follow(int16_t target_velo,int16_t current_velo);
int16_t PID_chasis_sway(int16_t target_velo,int16_t current_velo);
int16_t chasis_mode_shift(void);
int16_t spin_speed_change(float anger_chais_yaw);
void client_data_display(int16_t chasis_mode,u8 switch_shift,int16_t Adc_Volt);
extern int16_t Adc_Volt;
extern u8 flag_mode_shift_finish;
extern u8 switch_shift;

/*************Gimbal task*************/
#define EN_GIMBAL_TASK 1u

float Camera_PID_205_Position(float new_inc);
float Camera_PID_206_Position(float new_inc);
float PID_camera_yaw(float target_pos,float current_pos);  
float PID_camera_pitch(float target_pos,float current_pos);

extern int16_t output_208_bodan,output_205,output_206;
extern u8 newdata_flag,flag_Vw_Q;
extern int16_t AHRS_Yaw_angle;
extern float this_angle_Pitch,this_angle_Yaw,this_angle_Pitch_min,this_angle_Yaw_min,yaw_9050;
extern u8 flag_2006_start;
//extern float limitk_chasis;
void Gimbal_task(void *p_arg);
int16_t PID_205_Velocity(int16_t target_velo,int16_t current_velo);
int16_t PID_205_Position(float target_pos,float current_pos);
int16_t PID_205_buff_Velocity(int16_t target_velo,int16_t current_velo);
int16_t PID_205_buff_Position(float target_pos,float current_pos);
int16_t PID_205_Position_danzhou(float target_pos,float current_pos);
int16_t PID_206_Velocity(int16_t target_velo,int16_t current_velo);
int16_t PID_206_Position(float target_pos,float current_pos);
int16_t PID_206_buff_Velocity(int16_t target_velo,int16_t current_velo);
int16_t PID_206_buff_Position(float target_pos,float current_pos);
int16_t PID_207_Velocity(int16_t target_velo,int16_t current_velo);
int16_t PID_207_Position(float target_pos,float current_pos);
int16_t PID_205_calibrat_Position(float target_pos,float current_pos);
int16_t PID_206_calibrat_Position(float target_pos,float current_pos);

float PID_Gimbal_Pitch_Pos(float this_velo,float target_velo);
float PID_Gimbal_Pitch_Velo(float this_velo,float target_velo);
void spin(void);
void allow_gun_l(void);
void allow_gun_2(void);
float PID_Gimbal_YAW_Velo(float this_velo,float target_velo);
float PID_Gimbal_YAW_Pos(float this_velo,float target_velo);
float PID_Gimbal_YAW_Pos_Spin(float this_velo,float target_velo);
float PID_Gimbal_YAW_Velo_Spin(float this_velo,float target_velo);
float PID_camera_pitch_min(float target_pos,float current_pos);  
float PID_camera_yaw_min(float target_pos,float current_pos);  
float Camera_PID_206_Position_min(float new_inc);
float Camera_PID_205_Position_min(float new_inc);
float angle_gravity_compensate(float distance_z);
int16_t gimbal_mode_shift(void);

extern u8 flag_target_reached,flag_target_reached_buff;


/**************Gun task***************/
#define EN_GUN_TASK 1u
extern int16_t target_anger_bodan_208;
extern int16_t count_bullet;
extern u8 flag_friction_start;
extern u8 flag_E_right,flag_Q_left,flag_sway_ctrl,flag_Vw_C,flag_Vw_F,flag_Vw_R;
int16_t PID_201_Velocity_friction(int16_t target_velo,int16_t current_velo);
int16_t PID_202_Velocity_friction(int16_t target_velo,int16_t current_velo);
int16_t PID_208_Velocity_bodan(int16_t this_velo,int16_t target_velo);
int16_t PID_208_Position_bodan(int16_t target_velo,int16_t this_velo);
int16_t PID_204_Position(int16_t target_velo,int16_t this_velo);  
int16_t shoot_speed_choose(int16_t target_V201_friction,int16_t target_V202_friction);
extern u8 fire_flag;
extern uint16_t shooterHeat;
extern int16_t target_gun_velocity;
extern u8 speed_ready;
extern u8 flag_buff_start;
void Gun_task(void *p_arg);
int16_t shoot_mode_choose(void);
int16_t heat_17mm_limit(uint16_t heat0_limit,uint16_t heat,int16_t shoot_mode,int16_t shoot_speed,uint8_t robot_level);



void turnback_processing(int16_t mode);
void tmr1_callback(void *p_tmr, void *p_arg);



/*********************Small Gimbal task*******************************/
#define EN_GIMBAL_MIN_TASK 1u
void Gimbal_min_task(void *p_arg);



/*****************Time task***************/
#define EN_TIME_TASK 0u
void Time_task(void *p_arg);

/*************************Ω·Œ≤************************************/
#endif

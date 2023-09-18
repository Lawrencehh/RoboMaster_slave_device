#ifndef __PWM_H
#define __PWM_H

#include "stm32f4xx.h"

void TIM2_PWM_Init(void);
void TIM4_PWM_Init(void);
void TIM8_PWM_Init(void);
void TIM5_PWM_Init(void);
void Heat_PWM_Init(void);
extern int16_t angle1,angle2,angle3,angle4,angle;
extern int angle_1;
#endif

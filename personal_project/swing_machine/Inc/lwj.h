#ifndef __LWJ_H__
#define __LWJ_H__

#include "stm32l4xx_hal.h"

#define _0_DEGREE		45
#define _180_DEGREE 	225

enum
{
	idle ,forward, backward, left, right, forw_left, back_left, forw_right, back_right, swing
};

TIM_HandleTypeDef htim1;	//	4·û ¹ÙÄû Á¦¾î
TIM_HandleTypeDef htim2;	//	Servo motor Á¦¾î

void Wheel_Control(uint8_t wheel_dir, uint32_t PWM_Pulse);
void Swing_Contorl(uint8_t hit_flag);

#endif

#include "lwj.h"
#include "stm32l4xx_hal.h"

uint32_t PWM_Pulse = 800; // Default 최대값 설정, 추후 변경가능.
uint32_t wait = 0;
/*
 * 		함수 설명.
 *
 * 		받는 인자
 * 		1. 방향
 * 		2. pwm파형
 * 		이 값을 받아 Mechanic Wheel을 구동할 수 있도록 함.
 *
 * 		함수 사용법.
 *
 * 		Wheel_Control( 움직일 방향 , pwm 파형)
 *
 *
 * */

void Wheel_Control(uint8_t wheel_dir, uint32_t PWM_Pulse)
{
	// 정지 및 속도제어
	if(wheel_dir == idle)
	{
		PWM_Pulse = 0;
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 , PWM_Pulse);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2 , PWM_Pulse);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3 , PWM_Pulse);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4 , PWM_Pulse);
	}

	switch (wheel_dir)	//	방향에 따른 바퀴 포트 지정.
	{
	case forward	:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET);
		break;
	case backward	:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		break;
	case left		:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		break;
	case right		:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET);
		break;
	case forw_left	:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		break;
	case back_left	:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		break;
	case forw_right	:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET);
		break;
	case back_right	:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		break;
	}

}


/*
 * 		함수 설명.
 *
 * 		받는 인자
 * 		1. hit flag
 *
 *
 *
 * 		함수 사용법.
 *
 * 		Swing( hit flag )
 *
 * 		플레그에 따라 서보모터의 각도가 결정된다.
 *
 *
 * */

void Swing_Contorl(uint8_t hit_flag)
{
	if(hit_flag == swing)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, _180_DEGREE);
		HAL_Delay(500);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, _0_DEGREE);
	}
}



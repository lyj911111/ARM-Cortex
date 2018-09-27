#include "lwj.h"
#include "stm32l4xx_hal.h"

uint32_t PWM_Pulse = 800; // Default �ִ밪 ����, ���� ���氡��.
uint32_t wait = 0;
/*
 * 		�Լ� ����.
 *
 * 		�޴� ����
 * 		1. ����
 * 		2. pwm����
 * 		�� ���� �޾� Mechanic Wheel�� ������ �� �ֵ��� ��.
 *
 * 		�Լ� ����.
 *
 * 		Wheel_Control( ������ ���� , pwm ����)
 *
 *
 * */

void Wheel_Control(uint8_t wheel_dir, uint32_t PWM_Pulse)
{
	// ���� �� �ӵ�����
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

	switch (wheel_dir)	//	���⿡ ���� ���� ��Ʈ ����.
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
 * 		�Լ� ����.
 *
 * 		�޴� ����
 * 		1. hit flag
 *
 *
 *
 * 		�Լ� ����.
 *
 * 		Swing( hit flag )
 *
 * 		�÷��׿� ���� ���������� ������ �����ȴ�.
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



#ifndef INC_PWM_HPP_
#define INC_PWM_HPP_

#include <cstdint>

void PwmInit(uint16_t motor, uint16_t servo){

	//motor start
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	//servo start
//	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

	//motor init
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1, motor);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_2, motor);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_3, motor);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_4, motor);

	//servo init
//	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_1, servo);
//	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_2, servo);

	//初期化待機
	HAL_Delay(5000);
}

void PwmIdel(uint16_t motor, uint16_t servo){

	//motor idel
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1, motor);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_2, motor);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_3, motor);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_4, motor);

	//servo idel
//	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_1, servo);
//	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_2, servo);

}

void PwmGenerate(uint16_t* motor, uint16_t* servo){

	//motor idel
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1, motor[0]);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_2, motor[1]);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_3, motor[2]);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_4, motor[3]);

	//servo idel
//	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_1, servo[0]);
//	__HAL_TIM_SET_COMPARE(&htim12 , TIM_CHANNEL_2, servo[1]);

}

void PwmStop(){

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

	//	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
	//	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
}

#endif

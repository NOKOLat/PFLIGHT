#ifndef INC_PWM_HPP_
#define INC_PWM_HPP_

#include <cstdint>

//void PidtoPwm(uint16_t throttle, float control[3], uint16_t motor_pwm[4]){
//
//	motor_pwm[0] = (2500 * 0.45) + (throttle - 360) / (1680-360) * 250 + (control[0] + control[1] - control[2]);
//	motor_pwm[1] = (2500 * 0.45) + (throttle - 360) / (1680-360) * 250 + (control[0] - control[1] + control[2]);
//	motor_pwm[2] = (2500 * 0.45) + (throttle - 360) / (1680-360) * 250 - (control[0] + control[1] - control[2]);
//	motor_pwm[3] = (2500 * 0.45) + (throttle - 360) / (1680-360) * 250 - (control[0] - control[1] + control[2]);
//}

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
//	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1, servo);
//	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_2, servo);

}

void PwmGenerate(uint16_t* motor, uint16_t* servo){

	//motor idel
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1, motor[0]);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_2, motor[1]);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_3, motor[2]);
	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_4, motor[3]);

	//servo idel
//	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1, servo[0]);
//	__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_2, servo[1]);

}

void PwmStop(){


}

#endif

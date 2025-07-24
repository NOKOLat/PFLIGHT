/*
 * PWM.hpp
 *
 *  Created on: May 4, 2025
 *      Author: aoi25
 */

#ifndef INC_PWM_HPP_
#define INC_PWM_HPP_

#include "tim.h"
#include "flight_data.hpp"

/* モーター配置（上が前）
 * 	1	2
 *
 * 	3	4
 */

//モーターのタイマー番号
struct MotorTim{

	TIM_HandleTypeDef* motor1 = &htim1;
	TIM_HandleTypeDef* motor2 = &htim1;
	TIM_HandleTypeDef* motor3 = &htim1;
	TIM_HandleTypeDef* motor4 = &htim1;
};

//モーターのチャンネル番号
struct MotorChannel{

	uint32_t motor1 = TIM_CHANNEL_1;
	uint32_t motor2 = TIM_CHANNEL_2;
	uint32_t motor3 = TIM_CHANNEL_3;
	uint32_t motor4 = TIM_CHANNEL_4;
};

//サーボのタイマー番号
struct ServoTim{

	TIM_HandleTypeDef* servo1 = &htim12;
	TIM_HandleTypeDef* servo2 = &htim12;
};

//モーターチャンネル番号
struct ServoChannel{

	uint32_t servo1 = TIM_CHANNEL_1;
	uint32_t servo2 = TIM_CHANNEL_2;
};

//モーター用のPWM値
struct MotorPWM{

	uint16_t counter_period = 2499 + 1;

	uint16_t max  = counter_period * 0.95;
	uint16_t min  = counter_period * 0.45;
	uint16_t init = counter_period * 0.40;
};

//サーボー用のPWM値
struct ServoPWM{

	uint16_t counter_period = 2499 + 1;

	uint16_t open   = counter_period * 0.16;
	uint16_t center = counter_period * 0.40;
	uint16_t close  = counter_period * 0.72;
};

void CalcMotorPwm(float throttle, float control[3], uint16_t* motor);
void CalcServoPwm(SbusData sbus_data, uint16_t adc_value, uint16_t* servo_pwm);
void PwmInit();
void PwmIdel(uint16_t motor, uint16_t servo);
void PwmGenerate(uint16_t* motor, uint16_t* servo);
void PwmStop();

#endif /* INC_PWM_HPP_ */

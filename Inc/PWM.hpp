/*
 * PWM.hpp
 *
 *  Created on: May 4, 2025
 *      Author: aoi25
 */

#ifndef INC_PWM_HPP_
#define INC_PWM_HPP_

#include "tim.h"
#include "USER_Setting.hpp"

//モーター用のPWM値
struct MotorPWM{

	uint16_t counter_period = 2499 + 1;

	uint16_t max  = counter_period * 0.95;
	uint16_t min  = counter_period * 0.50;
	uint16_t init = counter_period * 0.40;

	uint16_t value[4] = {};
};

//サーボー用のPWM値
struct ServoPWM{

	uint16_t counter_period = 2499 + 1;

	uint16_t open   = counter_period * 0.16;
	uint16_t center = counter_period * 0.40;
	uint16_t close  = counter_period * 0.72;

	uint16_t value[2] = {};
};

void PidtoPwm(float throttle, float control[3], uint16_t* motor);
void PwmInit();
void PwmIdel(uint16_t motor, uint16_t servo);
void PwmGenerate(uint16_t* motor, uint16_t* servo);
void PwmStop();

#endif /* INC_PWM_HPP_ */

/*
 * LED.hpp
 *
 *  Created on: Jun 29, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_LED_HPP_
#define INC_LED_HPP_

#include "gpio.h"

enum class PinState: uint8_t{

	off = 0,
	on,
	toggle
};

void RedLed(PinState pin_state){

	if(PinState::off){

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	}
	else if(PinState::on){

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	}
	else{

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
	}
}

void GrrenLed(PinState pin_state){

	if(PinState::off){

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	}
	else if(PinState::on){

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	}
	else{

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	}
}

void YellowLed(PinState pin_state){

	if(PinState::off){

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	}
	else if(PinState::on){

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	}
	else{

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	}
}

#endif /* INC_LED_HPP_ */

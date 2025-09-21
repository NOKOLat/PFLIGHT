/*
 * LED.hpp
 *
 * LED制御クラスのヘッダファイル
 * UserSetting名前空間に定義してあるLEDSetting構造体を使用して、LEDの初期化と制御を行う
 *
 *  Created on: Jun 29, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_LED_HPP_
#define INC_LED_HPP_

#include <cstdint>
#include "gpio.h"

enum class PinState: uint8_t{

	off = 0,
	on,
	toggle
};


// LED制御クラス
class LED {

public:

	void LEDInit(GPIO_TypeDef* gpio_port, uint32_t gpio_pin){

		this->gpio_port = gpio_port;
		this->gpio_pin = gpio_pin;

		// 初期状態をoffに設定
		HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
	}

	void Set(PinState pin_state){

		if(pin_state == PinState::off){

			HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
		}
		else if(pin_state == PinState::on){

			HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
		}
		else{

			HAL_GPIO_TogglePin(gpio_port, gpio_pin);
		}
	}

private:

	GPIO_TypeDef* gpio_port;
	uint32_t gpio_pin;
};

#endif /* INC_LED_HPP_ */

/*
 * LEDSetting.hpp
 *
 * LEDの制御に使用するピンの設定
 * UserSetting名前空間に定義してあり、識別しやすいようにLEDSetting構造体にまとめている
 * 
 * 色ごとに定義してある（正直、役割ごとに分けたほうがいい: 例: armLed
 *
 *  Created on: Aug 24, 2025
 *      Author: aoi25
 */

#ifndef INC_LEDSETTING_HPP_
#define INC_LEDSETTING_HPP_

#include <cstdint>
#include "gpio.h"   

// LED設定構造体
struct LEDSetting {

    GPIO_TypeDef* gpio_port;
    uint32_t gpio_pin;
    GPIO_PinState pin_state;
};

// LED設定用のインスタンスの作成

namespace UserSetting{

	LEDSetting redLedSetting    = {GPIOC, GPIO_PIN_4, GPIO_PIN_RESET};
	LEDSetting yellowLedSetting = {GPIOB, GPIO_PIN_0, GPIO_PIN_RESET};
	LEDSetting greenLedSetting  = {GPIOB, GPIO_PIN_1, GPIO_PIN_RESET};
}

#endif /* INC_LEDSETTING_HPP_ */

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

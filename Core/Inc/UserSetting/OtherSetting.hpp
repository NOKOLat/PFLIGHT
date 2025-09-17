#ifndef OTHER_SETTING_HPP
#define OTHER_SETTING_HPP

#include "adc.h"    
#include "gpio.h"

namespace UserSetting{

	// リポチェックの有効化と電圧閾値の設定
	constexpr bool  lipo_check = true;
	constexpr float min_voltage = 7.7f;
	constexpr ADC_HandleTypeDef* lipo_check_adc = &hadc2; // ADCハンドルを指定

	// LEDとブザーの設定
	GPIO_TypeDef* lipo_check_led_port = GPIOA;
	uint16_t lipo_check_led_pin = GPIO_PIN_1;

	GPIO_TypeDef* lipo_check_buzzer_port = GPIOC;
	uint16_t lipo_check_buzzer_pin = GPIO_PIN_13;

}


#endif // OTHER_SETTING_HPP

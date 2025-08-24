/*
 * ImuSetting.hpp
 *
 *  Created on: Aug 24, 2025
 *      Author: aoi25
 */

#ifndef INC_USERSETTING_IMUSETTING_HPP_
#define INC_USERSETTING_IMUSETTING_HPP_

#include "gpio.h"
#include "spi.h"
#include "gpio.h"

struct ImuPinSetting{

	SPI_HandleTypeDef* spi_pin;
	GPIO_TypeDef* gpio_port;
	uint32_t gpio_pin;
};

namespace UserSetting{

	constexpr ImuPinSetting imuPinSetting{&hspi1, GPIOA, GPIO_PIN_4};
}

#endif /* INC_USERSETTING_IMUSETTING_HPP_ */

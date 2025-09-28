/*
 * ImuSetting.hpp
 *
 * IMUとの通信で使用するピンの設定
 * UserSetting名前空間に定義してあり、識別しやすいようにImuPinSetting構造体にまとめている
 *
 *  Created on: Aug 24, 2025
 *      Author: aoi25
 */

#ifndef INC_USERSETTING_IMUSETTING_HPP_
#define INC_USERSETTING_IMUSETTING_HPP_

#include "gpio.h"
#include "spi.h"

// SPI通信用のピン設定構造体
struct ImuPinSetting{

	SPI_HandleTypeDef* spi_pin;
	GPIO_TypeDef* gpio_port;
	uint32_t gpio_pin;
};

// 実際のIMUピン設定
// 実行中に変更できると危険+処理の高速化のためにconstexprで定義
namespace UserSetting{

	constexpr ImuPinSetting imu_pin_setting{&hspi1, GPIOA, GPIO_PIN_4};

	// キャリブレーションに使用するサンプル数
	// 多いほど精度が上がるが、時間がかかる (1000で約1.5秒程度)
	constexpr uint16_t calibration_count = 1000;
}

#endif /* INC_USERSETTING_IMUSETTING_HPP_ */

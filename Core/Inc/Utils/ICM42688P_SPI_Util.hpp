/* ICM42688P_SPI_Util.hpp
*
* ICM42688PのSPI通信ユーティリティ
* ICM42688P_HAL_SPIクラスをラップして、初期化、キャリブレーション、データ取得を簡単に行えるようにする
*
*  Created on: Sep 22, 2025
*      Author: aoi25
*/

#ifndef ICM42688P_SPI_UTIL_HPP
#define ICM42688P_SPI_UTIL_HPP

#include <cstdint>
#include <array>
#include <gpio.h>
#include "ICM42688P/ICM42688P_HAL_SPI.h"

class ICM42688P_SPI_Util {

public:

    ICM42688P_SPI_Util(SPI_HandleTypeDef* hspi, GPIO_TypeDef* gpio_port, uint32_t gpio_pin);
    
    uint8_t Init();
    uint8_t Calibration(uint16_t calibration_count);
    uint8_t GetData(std::array<float, 3>& accel_data, std::array<float, 3>& gyro_data);

private:

    ICM42688P_HAL_SPI icm;
};

#endif // ICM42688P_SPI_UTIL_HPP

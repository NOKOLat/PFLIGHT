/*
 * Dps368Setting.hpp
 *
 *  Created on: 2025-09-03
 *      Author: generated
 */

#ifndef INC_USERSETTING_DPS368SETTING_HPP_
#define INC_USERSETTING_DPS368SETTING_HPP_

#include "i2c.h"

struct Dps368I2cSetting{

    I2C_HandleTypeDef* i2c_pin;
};

namespace UserSetting{

    constexpr Dps368I2cSetting dps368I2cSetting{&hi2c1};
}

#endif /* INC_USERSETTING_DPS368SETTING_HPP_ */

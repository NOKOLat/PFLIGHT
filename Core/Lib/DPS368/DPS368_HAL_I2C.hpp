/*
 * DPS368_HAL_I2C.hpp
 *
 *  Created on: Aug 17, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_DPS368_HAL_I2C_HPP_
#define INC_DPS368_HAL_I2C_HPP_

#include "DPS368.hpp"
#include "DPS368_Register.hpp"
#include "i2c.h"

class DPS368_HAL_I2C : public DPS368{

	public:

		DPS368_HAL_I2C(I2C_HandleTypeDef* i2cPin);

	private:

		uint8_t RegWrite(DPS368_Register regAddr, uint8_t* txBuffer, uint8_t len) override;
		uint8_t RegRead(DPS368_Register regAddr, uint8_t* receiveBuffer, uint8_t dataLen) override;

		I2C_HandleTypeDef* i2cPin;
		uint8_t i2cAddr = 0x77 << 1;
};

#endif /* INC_DPS368_HAL_I2C_HPP_ */

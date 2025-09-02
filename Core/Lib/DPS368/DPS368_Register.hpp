/*
 * DPS368_Register.hpp
 *
 *  Created on: Aug 17, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_DPS368_REGISTER_HPP_
#define INC_DPS368_REGISTER_HPP_

#include <cstdint>

enum class DPS368_Register: uint8_t{

	PSR_B2 = 0x00,
	TMP_B2 = 0x03,
	PRS_CFG = 0x06,
	TMP_CFG = 0x07,
	MEAS_CFG = 0x08,
	CFG_REG  = 0x09,
	INT_STS  = 0x0A,
	FIFO_STS  = 0x0B,
	RESET = 0x0C,
	Product_ID = 0x0D,
	COEF = 0x10,
};

#endif /* INC_DPS368_REGISTER_HPP_ */

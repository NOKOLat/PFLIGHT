/*
 * DPS368.hpp
 *
 *  Created on: Aug 17, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_DPS368_HPP_
#define INC_DPS368_HPP_

#include "DPS368_Register.hpp"
#include "DPS368_Utility.hpp"

class DPS368{

	public:

		uint8_t init();
		uint8_t pressConfig(MEAS_RATE rate, MEAS_SAMPLING sampling);
		uint8_t tempConfig(MEAS_RATE rate, MEAS_SAMPLING sampling);
		uint8_t getPress();
		uint8_t getTemp();
		uint8_t getData(float * pressData, float * tempData);

	protected:

		virtual uint8_t RegRead(DPS368_Register regAddr, uint8_t* receiveBuffer, uint8_t dataLen) = 0;
		virtual uint8_t RegWrite(DPS368_Register regAddr, uint8_t* TransmitBuffer, uint8_t dataLen) = 0;

	private:

		void getTwosComplement(int32_t *raw, uint8_t length);
		int32_t pressCompensationScaleFactors = 524288;
		int32_t tempCompensationScaleFactors = 524288;

};



#endif /* INC_DPS368_HPP_ */

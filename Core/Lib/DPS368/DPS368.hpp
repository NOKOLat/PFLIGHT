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
		// PascalCase API: new method that uses pre-read coefficients
		uint8_t GetData(float * pressData, float * tempData);
		// compatibility wrapper for existing code
		uint8_t getData(float * pressData, float * tempData);

		// Read calibration coefficients once (call from main once)
		uint8_t ReadCoefficients();

	protected:

		virtual uint8_t RegRead(DPS368_Register regAddr, uint8_t* receiveBuffer, uint8_t dataLen) = 0;
		virtual uint8_t RegWrite(DPS368_Register regAddr, uint8_t* TransmitBuffer, uint8_t dataLen) = 0;

	private:

		void GetTwosComplement(int32_t *raw, uint8_t length);

		// calibration coefficients (kept as private members)
		int32_t m_c0_half;
		int32_t m_c1;
		int32_t m_c00;
		int32_t m_c10;
		int32_t m_c01;
		int32_t m_c11;
		int32_t m_c20;
		int32_t m_c21;
		int32_t m_c30;

		// Compensation scale factors
		int32_t press_compensation_scale_factors = 524288;
		int32_t temp_compensation_scale_factors = 524288;

};



#endif /* INC_DPS368_HPP_ */

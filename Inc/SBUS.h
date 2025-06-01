/*
 * SBUS.h
 *
 *  Created on: Oct 18, 2024
 *      Author: aoi25
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#include <cstdint>

class SBUS {

	public:

		void SetRawData(uint8_t raw_sbus[25]);
		void Encode();
		uint8_t IsSbus();
		uint8_t CheckFailsafe();
		void GetData(uint16_t sbus_data[10]);


	private:

		uint8_t raw_sbus[25] = {};
		uint16_t sbus_data[10] = {};
		uint8_t frame_lost = 0;
		uint8_t failsafe = 0;
		uint8_t encoding = 0;

};



#endif /* INC_SBUS_H_ */


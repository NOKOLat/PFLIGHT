/*
 * SBUS.h
 *
 *  Created on: Mar 28, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#include <cstdint>

class SBUS {

	public:

	uint8_t* GetBufferPointer();
	void Encode();
	uint8_t IsSBUS();
	void GetData(uint16_t* SBUSData);

	const uint8_t DataLen = 25;

	private:

	uint8_t RawData[25];
	uint16_t SBUSData[10] = {};
};

#endif /* INC_SBUS_H_ */

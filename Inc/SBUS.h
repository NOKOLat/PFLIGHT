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

	uint8_t encode(uint8_t sbus_buffer[25], uint16_t data_buffer[10]);

	private:

};

#endif /* INC_SBUS_H_ */


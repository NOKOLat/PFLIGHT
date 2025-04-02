/*
 * ring_buffer.h
 *
 *  Created on: Mar 31, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <cstdint>

class RINGBUFFER {

	public:

		void set_value(float Value[3]);
		float* get_value(uint8_t index);
		uint8_t get_index();

	private:


		uint8_t now_index = 0;
		uint8_t max_index = 16;
		float buffer[16][3] = {};
};

#endif /* INC_RING_BUFFER_H_ */

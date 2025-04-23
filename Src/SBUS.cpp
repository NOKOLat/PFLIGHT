/*
 * SBUS.cpp
 *
 *  Created on: Oct 18, 2024
 *      Author: aoi25
 */

#include <SBUS.h>

uint8_t SBUS::encode(uint8_t buffer[25], uint16_t sbus_data[10]){

	if(buffer[0] == 0x0F && buffer[25] == 0x00){

		sbus_data[0]  = 0;
		sbus_data[1]  = (buffer[1]        | buffer[2] << 8)   & 0x07FF;
		sbus_data[2]  = (buffer[2] >> 3   | buffer[3] << 5)   & 0x07FF;
		sbus_data[3]  = (buffer[3] >> 6   | buffer[4] << 2    | buffer[5] << 10) & 0x07FF;
		sbus_data[4]  = (buffer[5] >> 1   | buffer[6] << 7)   & 0x07FF;
		sbus_data[5]  = (buffer[6] >> 4   | buffer[7] << 4)   & 0x07FF;
		sbus_data[6]  = (buffer[7] >> 7   | buffer[8] << 1    | buffer[9] << 9) & 0x07FF;
		sbus_data[7]  = (buffer[9] >> 2   | buffer[10] << 6)  & 0x07FF;
		sbus_data[8]  = (buffer[10] >> 5  | buffer[11] << 3)  & 0x07FF;
		sbus_data[9]  = (buffer[12]       | buffer[13] << 8)  & 0x07FF;
		sbus_data[10] = (buffer[13] >> 3  | buffer[14] << 5)  & 0x07FF;

		return 0;
	}

	return 1;
}

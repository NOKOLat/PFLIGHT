/*
 * SBUS.cpp
 *
 *  Created on: Mar 28, 2025
 *      Author: Sezakiaoi
 */

#include "SBUS.h"

uint8_t* SBUS::GetBufferPointer(){

	return RawData;
}

uint8_t SBUS::IsSBUS(){

	if(RawData[0] == 0x0F && RawData[25] == 0x00){

		return 0;
	}

	return 1;
}

void SBUS::Encode(){

		SBUSData[0]  = (RawData[1]        | RawData[2] << 8)   & 0x07FF;
		SBUSData[1]  = (RawData[2] >> 3   | RawData[3] << 5)   & 0x07FF;
		SBUSData[2]  = (RawData[3] >> 6   | RawData[4] << 2    | RawData[5] << 10) & 0x07FF;
		SBUSData[3]  = (RawData[5] >> 1   | RawData[6] << 7)   & 0x07FF;
		SBUSData[4]  = (RawData[6] >> 4   | RawData[7] << 4)   & 0x07FF;
		SBUSData[5]  = (RawData[7] >> 7   | RawData[8] << 1    | RawData[9] << 9) & 0x07FF;
		SBUSData[6]  = (RawData[9] >> 2   | RawData[10] << 6)  & 0x07FF;
		SBUSData[7]  = (RawData[10] >> 5  | RawData[11] << 3)  & 0x07FF;
		SBUSData[8]  = (RawData[12]       | RawData[13] << 8)  & 0x07FF;
		SBUSData[9] = (RawData[13] >> 3  | RawData[14] << 5)  & 0x07FF;
}

void SBUS::GetData(uint16_t* SBUSData){

	for(uint8_t i=0; i<10; i++){

		SBUSData[i] = this->SBUSData[i];
	}
}

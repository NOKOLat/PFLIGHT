/*
 * SBUS.cpp
 *
 *  Created on: Oct 18, 2024
 *      Author: aoi25
 */

#include <SBUS.h>

void SBUS::SetRawData(uint8_t raw_sbus[25]){

	for(uint8_t i=0; i<25; i++){

		this->raw_sbus[i] = raw_sbus[i];
	}
}

void SBUS::GetData(uint16_t sbus_data[10]){

	for(uint8_t i=0; i<10; i++){

		sbus_data[i] = this->sbus_data[i];
	}
}

// 0: OK, 1:Error
uint8_t SBUS::IsSbus(){

	if((raw_sbus[0] == 0x0f) && (raw_sbus[24] == 0x00)){

		return 1;
	}

	return 0;
}

// 0: OK, 1:Failsafe
uint8_t SBUS::CheckFailsafe(){

	return (raw_sbus[23] != 0);
}

void SBUS::Encode(){

	if(raw_sbus[0] == 0x0F && raw_sbus[24] == 0x00){

		sbus_data[0]  = (raw_sbus[1]        | raw_sbus[2] << 8)   & 0x07FF;
		sbus_data[1]  = (raw_sbus[2] >> 3   | raw_sbus[3] << 5)   & 0x07FF;
		sbus_data[2]  = (raw_sbus[3] >> 6   | raw_sbus[4] << 2    | raw_sbus[5] << 10) & 0x07FF;
		sbus_data[3]  = (raw_sbus[5] >> 1   | raw_sbus[6] << 7)   & 0x07FF;
		sbus_data[4]  = (raw_sbus[6] >> 4   | raw_sbus[7] << 4)   & 0x07FF;
		sbus_data[5]  = (raw_sbus[7] >> 7   | raw_sbus[8] << 1    | raw_sbus[9] << 9) & 0x07FF;
		sbus_data[6]  = (raw_sbus[9] >> 2   | raw_sbus[10] << 6)  & 0x07FF;
		sbus_data[7]  = (raw_sbus[10] >> 5  | raw_sbus[11] << 3)  & 0x07FF;
		sbus_data[8]  = (raw_sbus[12]       | raw_sbus[13] << 8)  & 0x07FF;
		sbus_data[9]  = (raw_sbus[13] >> 3  | raw_sbus[14] << 5)  & 0x07FF;
	}
}

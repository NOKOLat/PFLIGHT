/*
 * ring_buffer.cpp
 *
 *  Created on: Mar 31, 2025
 *      Author: Sezakiaoi
 */

#include <ring_buffer.h>

void RINGBUFFER::set_value(float* Value){

	for(uint8_t i=0; i<3; i++){

		buffer[now_index][i] = Value[i];
	}

	if(now_index == 15){

		now_index = 0;
	}
	else{

		now_index ++;
	}
}

float* RINGBUFFER::get_value(uint8_t index){

	return buffer[index];
}

uint8_t RINGBUFFER::get_index(){

	return now_index;
}

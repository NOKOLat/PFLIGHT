/*
 * Debug.hpp
 *
 *  Created on: May 6, 2025
 *      Author: aoi25
 */

#ifndef INC_DEBUG_HPP_
#define INC_DEBUG_HPP_

#include <string>

void SendData(float data[3]){

	printf("%+4.4lf, %+4.4lf, %+4.4lf\n", data[0], data[1], data[2]);
}

void SendData(uint16_t data[4]){

	printf("%+4d, %+4d, %+4d, %+4d\n", data[0], data[1], data[2], data[3]);
}
#endif /* INC_DEBUG_HPP_ */

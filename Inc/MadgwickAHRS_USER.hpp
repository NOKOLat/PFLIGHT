/*
 * MadgwickAHRS_USER.hpp
 *
 *  Created on: May 27, 2025
 *      Author: aoi25
 */

#ifndef INC_MADGWICKAHRS_USER_HPP_
#define INC_MADGWICKAHRS_USER_HPP_

#include "MadgwickAHRS.h"

Madgwick madgwick;

float pre_angle[3] = {};

void Madgwick_Start(float rate){

	madgwick.begin(rate);
}

void Madgwick_UpDate(float accel[3], float gyro[3], float mag[3]){

	madgwick.update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}

void Madgwick_GetData(float angle[3], float angle_speed[3]){

	angle[0] = madgwick.getPitch();
	angle[1] = madgwick.getRoll();
	angle[2] = madgwick.getYaw();

	for(uint8_t i=0; i<3; i++){

		angle_speed[i] = (angle[i] - pre_angle[i]);
		pre_angle[i] = angle[i];
	}
}

#endif /* INC_MADGWICKAHRS_USER_HPP_ */

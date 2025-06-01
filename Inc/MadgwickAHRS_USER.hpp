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

void Madgwick_Start(float rate){

	madgwick.begin(rate);
}

void Madgwick_UpDate(float accel[3], float gyro[3], float mag[3]){

	madgwick.update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}

void Madgwick_GetAngle(float angle[3]){

	angle[0] = madgwick.getPitch();
	angle[1] = madgwick.getRoll();
	angle[2] = madgwick.getYaw();
}

#endif /* INC_MADGWICKAHRS_USER_HPP_ */

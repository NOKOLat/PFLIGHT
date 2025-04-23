/*
 * integration.h
 *
 *  Created on: Feb 21, 2025
 *      Author: takut
 */

#ifndef INC_CALCULATION_INTEGRATION_H_
#define INC_CALCULATION_INTEGRATION_H_

float integral(float raw,float dt){
	float k1 =  raw * dt;
	float k2 = (raw + 0.5 * k1) * dt;
	float k3 = (raw + 0.5 * k2) * dt;
	float k4 = (raw + k3) * dt;
	return  raw + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
}

#endif /* INC_CALCULATION_INTEGRATION_H_ */

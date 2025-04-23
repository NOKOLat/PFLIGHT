/*
 * position.h
 *
 *  Created on: Mar 21, 2025
 *      Author: takut
 */

#ifndef INC_CALCULATION_POSITION_H_
#define INC_CALCULATION_POSITION_H_

//#include "arm_math.h" //計算ライブラリdep
#include "matrix.h"
#include "integration.h"

#define G 9.80665

void position_cal(float AccelData[3],float speed[3],float position[3],float time,float rotation[3][3]){

	float inv[3][3];
	mat33(rotation,0,inv,'i');//回転行列の逆行列を求める
	float xyz[3];
	mat331(inv,AccelData,xyz,'*');
	speed[0] = integral(xyz[0],time);
	speed[1] = integral(xyz[1],time);


	position[0] = integral(speed[0],time);
	position[1] = integral(speed[1],time);

}


#endif /* INC_CALCULATION_POSITION_H_ */

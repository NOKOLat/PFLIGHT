/*
 * quaternion.h
 *
 *  Created on: Feb 20, 2025
 *      Author: takut
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

//#include "arm_math.h" //計算ライブラリdep

//加速度をクオータ二オンに変換
void quaternion(float accel[3],float q[4]){
	float norm;
	norm = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
	accel[0] /= norm; accel[1] /= norm; accel[2] /= norm;

	float thq = acos(accel[2]); //基底を(0,0,1)としたときの内積はz　入力値のノルムは1にしておく
	float cross[3] = {accel[1],accel[0],0};
	float cross_norm = sqrt(accel[0] * accel[0] + accel[1] * accel[1]);

	float  S = sin(thq/2),C = cos(thq/2);
	float angle[3] = {0,0,1};
	angle[0] = cross[0]/cross_norm;
	angle[1] = cross[1]/cross_norm;
	angle[2] = 0;
	q[0]= C;
	q[1]= angle[0]*S;
	q[2]= angle[1]*S;
	q[3]= 0;
}


//クオータ二オンを回転行列に変換
void quaternion_rotation (float quat[4],float roatation[3][3]){
	//　積の値を保存
	float qq[3][3]; //q[3] = 0
	qq[0][0] = quat[0]*quat[0];
	qq[0][1] = quat[0]*quat[1]; qq[0][2] = quat[0]*quat[2];
	qq[1][1] = quat[1]*quat[1]; qq[1][2] = quat[1]*quat[2];
	qq[2][1] = quat[2]*quat[1]; qq[2][2] = quat[2]*quat[2];

	//回転行列を計算
	roatation[0][0] = qq[0][0] + qq[1][1] - qq[2][2];
	roatation[0][1] = 2 * qq[1][2];
	roatation[0][2] = 2 * qq[0][2];
	roatation[1][0] = 2 * qq[1][2];
	roatation[1][1] = qq[0][0] - qq[1][1] + qq[2][2];
	roatation[1][2] = 2 * qq[0][1] ;
	roatation[2][0] = 2 * qq[0][2] ;
	roatation[2][1] = 2 * qq[0][1] ;
	roatation[2][2] = qq[0][0] - qq[1][1] - qq[2][2];
}

//現在保存されている回転行列をオイラー角に変換
void euler (float theta[3],float rotation[3][3]){
	theta[0] = asin(rotation[2][1]);
	theta[1] = asin(-rotation[0][2]);

	//弧度法から度数法に変換
	theta[0] *=(-180/M_PI);
	theta[1] *=(-180/M_PI);

}

#endif /* INC_QUATERNION_H_ */

/*
 * kalman_filter.h
 *
 *  Created on: Sep 28, 2024
 *      Author: takut
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#define DEBUG_MODE 1

#define  G 9.80665

//#include "arm_math.h" //計算ライブラリdep
#include "quaternion.h"
#include "matrix.h"

//#include "stdio.h"

uint8_t setting_theta (float AccelData[3],float GyroData[3],float time,
		float q_kalman[4],float q_obs[4],float q_pre[4],float A[4][4],float Q[4][4],float R[4][4]);


void kalman_theta(float AccelData[3],float GyroData[3],float theta[3],float time,float rotation[3][3]){
	static float Q[4][4];//予測ノイズ
	static float R[4][4]={};//観測値ノイズ
	static float A[4][4]={};//ヤコビアン行列
	static float At[4][4]={};//Aの転置
	static float SQ[4][4]={};//予測分散
	static float SR[4][4]={};//観測分散
	static float S[4][4]={};//推定対象の分散
	static float I[4][4]//単位行列
		  ={ {1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1} };
	static float K[4][4]={};//カルマンゲイン

	static float q_obs[4],q_pre[4],q_kalman[4]={1,0,0,0};//クォータニオン


	setting_theta(AccelData,GyroData,time,q_kalman,q_obs,q_pre,A,Q,R);
	theta[2] = GyroData[2]/255*45;//適宜調節



	if (DEBUG_MODE){
		quaternion_rotation(q_obs,rotation);//qの回転行列がroatに入る
		euler(theta,rotation);//現在保存されている回転行列をオイラー角に変換する
		//printf("%+.4f %+.4f %+.4f ",theta[0],theta[1],theta[2]);
	}


	//SQ = A * S * A.transpose() + Q;
	float buf[4][4];
	mat44(A,0,At,'t');
	mat444(A,S,At,buf);
	mat44(buf,Q,SQ,'+');
	if (SQ[0][0]>1000){//発散防止
		SQ[0][0] = 1000;
		SQ[1][1] = 1000;
		SQ[2][2] = 1000;
		SQ[3][3] = 1000;
	}
	//printf("SQ %.2f ",SQ[0][0]);

	//SR = SQ + R
	mat44(SQ,R,SR,'+');

	//K = SQ * C.transpose() * SR.inverse();
	mat44(SR,0,buf,'i');
	mat444(SQ,I,buf,K);



	//th.kalman = th.pre + K * (th.obs - th.pre);
	float buf1[4],buf2[4];
	mat41(q_obs,q_pre,buf1,'-');
	mat441(K,buf1,buf2,'*');
	mat41(q_pre,buf2,q_kalman,'+');
	q_kalman[3]=0;

	//正規化
	float norm;
	norm = 1/sqrt (q_kalman[0]*q_kalman[0] + q_kalman[1]*q_kalman[1] + q_kalman[2]*q_kalman[2]);
	q_kalman[0] *= norm;
	q_kalman[1] *= norm;
	q_kalman[2] *= norm;
	q_kalman[3] *= norm;

	//S = (I - K * C) * SQ;
	mat44(I,K,buf,'-');
	mat44(buf,SQ,S,'*');

	quaternion_rotation(q_kalman,rotation);//qの回転行列がroatに入る
	euler(theta,rotation);//現在保存されている回転行列をオイラー角に変換する


}


//カルマンフィルタ設定
uint8_t setting_theta (float AccelData[3],float GyroData[3],float time,float q_kalman[4],float q_obs[4],float q_pre[4],float A[4][4],float Q[4][4],float R[4][4]){

	float noise = sqrt( GyroData[0]*GyroData[0] + GyroData[1]*GyroData[1] )/500*10;


	//printf("noise %+.2f ",noise);

	q_pre[0] = q_kalman[0] + ( -GyroData[0]*q_obs[1] - GyroData[1]*q_obs[2] - GyroData[2]*q_obs[3] )*0.5 *time ;
	q_pre[1] = q_kalman[1] + (  GyroData[0]*q_obs[0] + GyroData[2]*q_obs[2] - GyroData[1]*q_obs[3] )*0.5 *time ;
	q_pre[2] = q_kalman[2] + (  GyroData[1]*q_obs[0] - GyroData[2]*q_obs[1] + GyroData[0]*q_obs[3] )*0.5 *time ;
	q_pre[3] = 0;

	A[0][0] =        0         ; A[0][1] = -GyroData[0] * 0.5; A[0][2] = -GyroData[1] * 0.5; A[0][3] = -GyroData[2] * 0.5;
	A[1][0] = GyroData[0] * 0.5; A[1][1] =        0          ; A[1][2] =  GyroData[2] * 0.5; A[1][3] = -GyroData[1] * 0.5;
	A[2][0] = GyroData[1] * 0.5; A[2][1] = -GyroData[2] * 0.5; A[2][2] =        0          ; A[2][3] =  GyroData[0] * 0.5;
	A[3][0] = GyroData[2] * 0.5; A[3][1] =  GyroData[1] * 0.5; A[3][2] = -GyroData[0] * 0.5; A[3][3] =         0         ;

	Q[0][0] = noise; Q[1][1] = noise; Q[2][2] = noise; Q[3][3] = noise;
	R[0][0] = noise; R[1][1] = noise; R[2][2] = noise; R[3][3] = noise;

	float norm = sqrt( AccelData[0]*AccelData[0] + AccelData[1]*AccelData[1] + AccelData[2]*AccelData[2] );
		//printf("%3.2f ",norm);

	if (abs(norm -G) < 0.1*G){
		quaternion(AccelData,q_obs);//加速度をクォータ二オンに変換　クオータ二オンがqに入る

		return 0;

	}else{
		q_obs[0] = q_kalman[0];
		q_obs[1] = q_kalman[1];
		q_obs[2] = q_kalman[2];
		q_obs[3] = 0;
		return 1;

	}

}


#endif /* INC_KALMAN_H_ */

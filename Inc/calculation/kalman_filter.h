/*
 * kalman_filter.h
 *
 *  Created on: Sep 28, 2024
 *      Author: takut
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#define pre_Noise 0.05
#define obs_Noise 0.05

//#include "arm_math.h" //計算ライブラリdep
#include "quaternion.h"
#include "matrix.h"

//#include "stdio.h"

void setting_theta (float AccelData[3],float GyroData[3],float time,
		float q_kalman[4],float q_obs[4],float q_pre[4],float A[4][4],float Q[4][4],float R[4][4]);

void setting_position (float AccelData[3],float GyroData[3],float time,
		float pre[3],float obs[3],float A[3][3],float Q[3][3],float R[3][3]);


void kalman_theta(float AccelData[3],float GyroData[3],float theta[3],float time,float rotation[3][3]){
	static float Q[4][4];//予測ノイズ
	static float R[4][4];//観測値ノイズ
	static float A[4][4];//ヤコビアン行列
	static float At[4][4];//Aの転置
	static float SQ[4][4];//予測分散
	static float SR[4][4];//観測分散
	static float S[4][4];//推定対象の分散
	static float I[4][4]//単位行列
		  ={ {1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1} };
	static float K[4][4];//カルマンゲイン

	static float q_obs[4],q_pre[4],q_kalman[4]={1,0,0,0};//クォータニオン

	setting_theta(AccelData,GyroData,time,q_kalman,q_obs,q_pre,A,Q,R);
	quaternion_rotation(q_obs,rotation);//qの回転行列がroatに入る
	euler(theta,rotation);//現在保存されている回転行列をオイラー角に変換する
	//printf("%f ",theta[0]);


	//SQ = A * S * A.transpose() + Q;
	float buf[4][4];
	mat44(A,0,At,'t');
	mat444(A,S,At,buf);
	mat44(buf,Q,SQ,'+');

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
	norm = sqrt (q_kalman[0]*q_kalman[0] + q_kalman[1]*q_kalman[1] + q_kalman[2]*q_kalman[2]);
	q_kalman[0] /= norm;
	q_kalman[1] /= norm;
	q_kalman[2] /= norm;
	q_kalman[3] /= norm;

	//S = (I - K * C) * SQ;
	mat44(I,K,buf,'-');
	mat44(buf,SQ,S,'*');

	quaternion_rotation(q_kalman,rotation);//qの回転行列がroatに入る
	euler(theta,rotation);//現在保存されている回転行列をオイラー角に変換する
	//printf("%f %f\n",theta[0],theta[0]);
}


//カルマンフィルタ設定
void setting_theta (float AccelData[3],float GyroData[3],float time,float q_kalman[4],float q_obs[4],float q_pre[4],float A[4][4],float Q[4][4],float R[4][4]){
	quaternion(AccelData,q_obs);//加速度をクォータ二オンに変換　クオータ二オンがqに入る

	q_pre[0] = q_kalman[0] +( -GyroData[0]*q_obs[1] - GyroData[1]*q_obs[2] - GyroData[2]*q_obs[3] )*0.5 *time;
	q_pre[1] = q_kalman[1] +(  GyroData[0]*q_obs[0] + GyroData[2]*q_obs[2] - GyroData[1]*q_obs[3] )*0.5 *time;
	q_pre[2] = q_kalman[2] +(  GyroData[1]*q_obs[0] - GyroData[2]*q_obs[1] + GyroData[0]*q_obs[3] )*0.5 *time;
	q_pre[3] = 0;

	A[0][0] =        0         ; A[0][1] = -GyroData[0] * 0.5; A[0][2] = -GyroData[1] * 0.5; A[0][3] = -GyroData[2] * 0.5;
	A[1][0] = GyroData[0] * 0.5; A[1][1] =        0          ; A[1][2] =  GyroData[2] * 0.5; A[1][3] = -GyroData[1] * 0.5;
	A[2][0] = GyroData[1] * 0.5; A[2][1] = -GyroData[2] * 0.5; A[2][2] =        0          ; A[2][3] =  GyroData[0] * 0.5;
	A[3][0] = GyroData[2] * 0.5; A[3][1] =  GyroData[1] * 0.5; A[3][2] = -GyroData[0] * 0.5; A[3][3] =         0         ;

	Q[0][0] = pre_Noise; Q[1][1] = pre_Noise; Q[2][2] = pre_Noise; Q[3][3] = pre_Noise;
	R[0][0] = obs_Noise; R[1][1] = obs_Noise; R[2][2] = obs_Noise; R[3][3] = obs_Noise;

}



void kalman_position(float AccelData[3],float GyroData[3],float time){
	static float Q[3][3];//予測ノイズ
	static float R[3][3];//観測値ノイズ
	static float A[3][3];//ヤコビアン行列
	static float At[3][3];//Aの転置
	static float SQ[3][3];//予測分散
	static float SR[3][3];//観測分散
	static float S[3][3];//推定対象の分散
	static float I[3][3]//単位行列
		  ={ {1,0,0},{0,1,0},{0,0,1}};
	static float K[3][3];//カルマンゲイン

	static float pre[3],obs[3],kalman[3];


	setting_position(AccelData,GyroData,time,pre,obs,A,Q,R);

	//SQ = A * S * A.transpose() + Q;
	float buf[3][3];
	mat33(A,0,At,'t');
	mat333(A,S,At,buf);
	mat33(buf,Q,SQ,'+');

	//SR = SQ + R
	mat33(SQ,R,SR,'+');

	//K = SQ * C.transpose() * SR.inverse();
	mat33(SR,0,buf,'i');
	mat333(SQ,I,buf,K);

	//th.kalman = th.pre + K * (th.obs - th.pre);
	float buf1[3],buf2[3];
	mat31(obs,pre,buf1,'-');
	mat331(K,buf1,buf2,'*');
	mat31(pre,buf2,kalman,'+');

	//S = (I - K * C) * SQ;
	mat33(I,K,buf,'-');
	mat33(buf,SQ,S,'*');
}

void setting_position (float AccelData[3],float GyroData[3],float time,float pre[3],float obs[3],float A[3][3],float Q[3][3],float R[3][3]){
	pre[0] = ( -GyroData[0]*obs[1] - GyroData[1]*obs[2] - GyroData[2]*obs[3] )*0.5 *time;
	pre[1] = (  GyroData[0]*obs[0] + GyroData[2]*obs[2] - GyroData[1]*obs[3] )*0.5 *time;
	pre[2] = (  GyroData[1]*obs[0] - GyroData[2]*obs[1] + GyroData[0]*obs[3] )*0.5 *time;
	pre[3] = (  GyroData[2]*obs[0] + GyroData[1]*obs[1] - GyroData[0]*obs[2] )*0.5 *time;

	A[0][0] =        0         ; A[0][1] = -GyroData[0] * 0.5; A[0][2] = -GyroData[1] * 0.5; A[0][3] = -GyroData[2] * 0.5;
	A[1][0] = GyroData[0] * 0.5; A[1][1] =        0          ; A[1][2] =  GyroData[2] * 0.5; A[1][3] = -GyroData[1] * 0.5;
	A[2][0] = GyroData[1] * 0.5; A[2][1] = -GyroData[2] * 0.5; A[2][2] =        0          ; A[2][3] =  GyroData[0] * 0.5;
	A[3][0] = GyroData[2] * 0.5; A[3][1] =  GyroData[1] * 0.5; A[3][2] = -GyroData[0] * 0.5; A[3][3] =         0         ;

	Q[0][0] = pre_Noise; Q[1][1] = pre_Noise; Q[2][2] = pre_Noise;
	R[0][0] = obs_Noise; R[1][1] = obs_Noise; R[2][2] = obs_Noise;
}
#endif /* INC_KALMAN_H_ */

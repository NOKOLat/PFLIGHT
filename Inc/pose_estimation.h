/*
 * pose_estimation.h
 *
 *  Created on: Mar 21, 2025
 *      Author: takut
 */

#ifndef INC_POSE_ESTIMATION_H_
#define INC_POSE_ESTIMATION_H_

#include "arm_math.h" //計算ライブラリdep
#include "calculation/FIR.h" //FIRフィルタ
#include "calculation/kalman_filter.h" //カルマンフィルタ
#include "calculation/integration.h"
#include "calculation/position.h"
#include "stdio.h"

uint8_t PoseEstimation(float AccelData[3],float GyroData[3],float MagData[3],float theta[3],float speed[3],float position[3]){
	static uint8_t i = 0;
	static float sum[3]={};

	if (i<BLOCK_SIZE/4){
		FIR_set(GyroData);//観測値を保存する

		sum[0] += AccelData[0]; sum[1] +=  AccelData[1]; sum[2] +=  AccelData[2];
		i++;

		return 1; //データを取得中

	}else{

		 AccelData[0] = sum[0]/i;
		 AccelData[1] = sum[1]/i;
		 AccelData[2] = sum[2]/i;
		 sum[0] = 0;
		 sum[1] = 0;
		 sum[2] = 0;
		i = 0;

		FIR_calc(GyroData);//保存されたデータにFIRをかけ、引数に出力する

		float rotation[3][3];
		kalman_theta(AccelData,GyroData,theta,0.0005,rotation);//カルマンフィルタ

		theta[2] = GyroData[2];
		//position_cal(AccelData,speed,position,(end-start)*1.0e-5,rotation);

		//theta[2] += integral(GyroData[2],0.001);


		return 0; //計算処理終了
	}

}



#endif /* INC_POSE_ESTIMATION_H_ */

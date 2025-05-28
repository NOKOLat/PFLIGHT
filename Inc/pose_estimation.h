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

#define RE 30
static double alpha;//平均化定数


void Avg(float data[3],float avg[3]){
    uint8_t i;
    static uint64_t n;
    static float sum[3];
    static float buf[3];

    //平均化
    n ++;
    alpha = ( 2.0/(n+1) );//1以下になるようにするn=1,2,3,・・・
    for (i=0;i<3;i++){

        if (n>RE-(RE/10)){
            sum[i] += data[i];
        }else{
            sum[i] = 0;
        }

        if (n == RE){
            avg[i] = sum[i]/(RE/10);

        }else{
            avg[i] = (buf[i])+alpha*( (data[i])-(buf[i]) );
        }
        buf[i] = avg[i];
    }
    if (n == RE){
        n = 1;
    }

}

uint8_t PoseEstimation(float AccelData[3],float GyroData[3],float theta[3]){
	static uint8_t i = 0;
	static float sum[3]={};

	if (i < BLOCK_SIZE/4){
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

		//printf("%+3.3lf %+3.3lf %+3.3lf ", AccelData[0], AccelData[1], AccelData[2]);

		FIR_calc(GyroData);//保存されたデータにFIRをかけ、引数に出力する

		//printf("%+3.3lf %+3.3lf %+3.3lf ", GyroData[0], GyroData[1], GyroData[2]);

		float rotation[3][3];
		kalman_theta(AccelData,GyroData,theta,0.00125,rotation);//カルマンフィルタ

		Avg(theta,theta);
		if (DEBUG_MODE){
			printf("%+3.3lf %+3.3lf %+3.3lf\n", theta[0], theta[1], theta[2]);
		}



		return 0; //計算処理終了
	}

}



#endif /* INC_POSE_ESTIMATION_H_ */

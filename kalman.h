/*
 * kalman.h
 *
 *  Created on: Aug 12, 2025
 *      Author: takut
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "main.h"
#include "matrix.h"
#include <cstdint>


// フィルタの最大サイズ定義
#define STATE_SIZE_MAX    4    // 状態ベクトルの最大サイズ（必要に応じて拡張）
#define OBS_SIZE_MAX      4    // 観測ベクトルの最大サイズ（必要に応じて拡張）

// 実際のサイズはインスタンスごとに指定（例: constructor等で）

class Kalman {
public:
    void Init(uint8_t state_size, uint8_t obs_size);
    void Update();
    void GetData(float* out_states);

    // 入力パラメータ
    float observation[OBS_SIZE_MAX];                // 観測値
    float prediction[STATE_SIZE_MAX];               // 予測値
    float output[STATE_SIZE_MAX];                   // 出力値
    float observation_noise;                        // 観測ノイズ
    float prediction_noise;                         // 予測ノイズ
    float system_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX];    // F
    float observation_matrix[OBS_SIZE_MAX * STATE_SIZE_MAX]; // H

   

private:
    bool is_initialized;

    uint8_t STATE_SIZE = 0;
    uint8_t OBS_SIZE = 0;

    
    // 保存する変数
    float prediction_noise_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX];      // Q
    float observation_noise_matrix[OBS_SIZE_MAX * OBS_SIZE_MAX];         // R
    float prediction_covariance[STATE_SIZE_MAX * STATE_SIZE_MAX];        // P
    float kalman_gain[STATE_SIZE_MAX * OBS_SIZE_MAX];                   // K
    float identity_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX];             // I

    // 必要最低限の一時バッファ
    float temp[STATE_SIZE_MAX * STATE_SIZE_MAX]; 
    float temp1[STATE_SIZE_MAX * OBS_SIZE_MAX];
    float temp2[OBS_SIZE_MAX * STATE_SIZE_MAX];
    float temp3[OBS_SIZE_MAX * OBS_SIZE_MAX];
    
    MATRIX P ,Q , R , K, I , F , H , Z ,
    PRE , OUT, TEMP, TEMP1, TEMP2, TEMP3;

};

#endif /* INC_KALMAN_H_ */

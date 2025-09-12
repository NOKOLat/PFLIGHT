/*
 * kalman.hpp
 *
 *  Created on: Aug 21, 2025
 *      Author: takut
 */

#ifndef INC_KALMAN_FILTER_HPP_
#define INC_KALMAN_FILTER_HPP_

#include <cstdint>
#include "math.h"


// EWMA alpha for noise estimation
#define NOISE_ALPHA 0.02f

// 簡易スムーザー有効/無効
#define KALMAN_USE_SMOOTHER false

// Smoother tuning (fixed alpha)
// 単一の固定αを使う。0.0f に近いほど強い平滑化、1.0f に近いほど追従性が良い。
// 値は 0..1 に設定してください。
#define SMOOTHER_ALPHA 0.8f

// スムーザーの履歴長（移動平均を使う場合）。1 にすると移動平均は無効（EMAパス）。
// 増やすと安定性は上がるが遅延は (N-1)/2 サンプル分増加します。
#define SMOOTHER_WINDOW_SIZE 4


// フィルタの最大サイズ定義
#define STATE_SIZE_MAX    2    // 状態ベクトルの最大サイズ（必要に応じて拡張）
#define OBS_SIZE_MAX      1    // 観測ベクトルの最大サイズ（必要に応じて拡張）

// テンポラリバッファの最大サイズ（state x state が最大想定）
#define TEMP_BUFFER_MAX (STATE_SIZE_MAX * STATE_SIZE_MAX)

// 実際のサイズはインスタンスごとに指定（例: constructor等で）


class KalmanFilter {
public:
    void Init(uint8_t state_size, uint8_t obs_size);
    void Update();
    // Estimate measurement/prediction noise from incoming measurements (EWMA)
    void EstimateNoise(float meas);
    void GetData(float* out_states);

    // 入力パラメータ
    float observation[OBS_SIZE_MAX]={};                // 観測値
    float prediction[STATE_SIZE_MAX]={};               // 予測値


    float system_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX]={};    // F
    float observation_matrix[OBS_SIZE_MAX * STATE_SIZE_MAX]={}; // H



private:
    uint8_t STATE_SIZE = 0;
    uint8_t OBS_SIZE = 0;

    // 保存する変数
    float observation_noise = 0.0f;                         // 観測ノイズ
    float prediction_noise = 0.0f;                          // 予測ノイズ
    float output[STATE_SIZE_MAX]={};                        // 出力値
    float prediction_noise_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX] = {};      // Q
    float observation_noise_matrix[OBS_SIZE_MAX * OBS_SIZE_MAX] = {};         // R
    float prediction_covariance[STATE_SIZE_MAX * STATE_SIZE_MAX] = {};        // P
    float observation_covariance[OBS_SIZE_MAX * OBS_SIZE_MAX] = {};          // SO
    float kalman_gain[STATE_SIZE_MAX * OBS_SIZE_MAX] = {};                   // K
    float identity_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX] = {};             // I

    // 汎用テンポラリバッファ（ヘッダでは最大サイズで確保）
    float temp_buffer_a[TEMP_BUFFER_MAX] = {};
    float temp_buffer_b[TEMP_BUFFER_MAX] = {};

    #if KALMAN_USE_SMOOTHER
    float smooth_output[STATE_SIZE_MAX]={};            // 平滑化後の出力
    float smooth_buffer[STATE_SIZE_MAX][SMOOTHER_WINDOW_SIZE] = {};       // 直近N回分のバッファ（移動平均用）
    int smooth_index = 0;

    // スムーザー関数
    void Smoother();
    #endif

};



#endif /* INC_KALMAN_FILTER_HPP_ */

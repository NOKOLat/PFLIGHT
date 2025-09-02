/*
 * kalman.cpp
 *
 *  Created on: Aug 12, 2025
 *      Author: takut
 */

#include "kalman_filter.hpp"
#include <cstdint>
#include "math.h"


void KalmanFilter::Init(uint8_t state_size, uint8_t obs_size) {

    STATE_SIZE = state_size;
    OBS_SIZE = obs_size;

    // 各MATRIXメンバにdata/rows/colsをセット
    P.data = prediction_covariance;     P.rows = state_size;    P.cols = state_size;
    Q.data = prediction_noise_matrix;   Q.rows = state_size;    Q.cols = state_size;
    R.data = observation_noise_matrix;  R.rows = obs_size;      R.cols = obs_size;
    SO.data = observation_covariance;   SO.rows = obs_size;    SO.cols = obs_size;
    K.data = kalman_gain;               K.rows = state_size;    K.cols = obs_size;
    I.data = identity_matrix;           I.rows = state_size;    I.cols = state_size;
    F.data = system_matrix;             F.rows = state_size;    F.cols = state_size;
    H.data = observation_matrix;        H.rows = obs_size;      H.cols = state_size;
    Z.data = observation;               Z.rows = obs_size;      Z.cols = obs_size;
    PRE.data = prediction;              PRE.rows = state_size;  PRE.cols = 1;
    OUT.data = output;                  OUT.rows = state_size;  OUT.cols = obs_size;
    // 汎用テンポラリはヘッダで確保されたバッファを使用
    TEMP_A.data = temp_buffer_a;    // ヘッダで定義された最大サイズのバッファ
    TEMP_B.data = temp_buffer_b;
    // 初期は最大サイズをセット（Update内で必要な rows/cols を上書きする）
    TEMP_A.rows = STATE_SIZE_MAX; TEMP_A.cols = STATE_SIZE_MAX;
    TEMP_B.rows = STATE_SIZE_MAX; TEMP_B.cols = STATE_SIZE_MAX;

    // ゼロクリア
    MatInit(&P);
    MatInit(&Q);
    MatInit(&R);
    MatInit(&K);
    MatInit(&I);
    MatInit(&F);
    MatInit(&H);
    MatInit(&Z);
    MatInit(&PRE);
    MatInit(&OUT);
    // initialize generic temporaries
    MatInit(&TEMP_A);
    MatInit(&TEMP_B);


    // P, I を対角1で初期化
    for (uint8_t i = 0; i < state_size; i++) {
        prediction_covariance[i * state_size + i] = 1.0f;
        identity_matrix[i * state_size + i] = 1.0f;
    }

#if KALMAN_USE_SMOOTHER
    // スムーザー用バッファ初期化
    for (uint8_t i = 0; i < STATE_SIZE_MAX; i++) {
        for (uint8_t j = 0; j < SMOOTHER_WINDOW_SIZE; j++) {
            smooth_buffer[i][j] = 0.0f;
        }
        smooth_output[i] = 0.0f;
    }
    smooth_index = 0;
#endif

}


void KalmanFilter::Update() {

    // Q, R は Init() でゼロ初期化済みと仮定し、ここでは対角要素のみ更新
    for (uint8_t i = 0; i < STATE_SIZE; ++i) {
        prediction_noise_matrix[i * STATE_SIZE + i] = prediction_noise;
    }
    for (uint8_t i = 0; i < OBS_SIZE; ++i) {
        observation_noise_matrix[i * OBS_SIZE + i] = observation_noise;
    }

    // ---------- P = F * P * F^T + Q ----------
    // TEMP_A := F * P  (state x state)
    TEMP_A.rows = STATE_SIZE; TEMP_A.cols = STATE_SIZE;
    MatCalc(&F, &P, &TEMP_A, '*');        // TEMP_A = F * P

    // TEMP_B := F^T  (state x state)
    TEMP_B.rows = STATE_SIZE; TEMP_B.cols = STATE_SIZE;
    MatCalc(&F, nullptr, &TEMP_B, 't');  // TEMP_B = F^T

    // P := TEMP_A * TEMP_B  (state x state)
    MatCalc(&TEMP_A, &TEMP_B, &P, '*');    // P = (F*P) * F^T

    // P := P + Q
    MatCalc(&P, &Q, &P, '+');

    // ---------- SO = H * P * H^T + R ----------
    // TEMP_A := H * P  (obs x state)
    TEMP_A.rows = OBS_SIZE; TEMP_A.cols = STATE_SIZE;
    MatCalc(&H, &P, &TEMP_A, '*');       // TEMP_A = H * P

    // TEMP_B := H^T  (state x obs)
    TEMP_B.rows = STATE_SIZE; TEMP_B.cols = OBS_SIZE;
    MatCalc(&H, nullptr, &TEMP_B, 't');  // TEMP_B = H^T

    // TEMP_A := TEMP_A * TEMP_B  (obs x obs) -> SO
    TEMP_A.rows = OBS_SIZE; TEMP_A.cols = STATE_SIZE;
    MatCalc(&TEMP_A, &TEMP_B, &SO, '*'); // SO = H * P * H^T

    // SO := SO + R
    MatCalc(&SO, &R, &SO, '+');

    // ---------- K = P * H^T * SO^-1 ----------
    // INV_SO := inverse(TEMP_A) (obs x obs) - store into TEMP_A (overwrite)
    TEMP_A.rows = OBS_SIZE; TEMP_A.cols = OBS_SIZE;
    MatCalc(&SO, nullptr, &TEMP_A, 'i'); // TEMP_A = (SO)^-1

    // TEMP_B := P * H^T  (state x obs)
    TEMP_B.rows = STATE_SIZE; TEMP_B.cols = OBS_SIZE;
    MatCalc(&P, &TEMP_B, &TEMP_B, '*');    // TEMP_B = P * H^T

    // K := TEMP_B * TEMP_A  (state x obs)
    K.rows = STATE_SIZE; K.cols = OBS_SIZE;
    MatCalc(&TEMP_B, &TEMP_A, &K, '*');    // K = P * H^T * (SO)^-1

    // ---------- output = prediction + K * (observation - H * prediction) ----------

    // TEMP_A := H * PRE  (obs x 1)
    TEMP_A.rows = OBS_SIZE; TEMP_A.cols = 1;
    MatCalc(&H, &PRE, &TEMP_A, '*');     // TEMP_A = H * prediction

    // TEMP_B used as vector holder: TEMP_B := Z - TEMP_A  (obs x 1)
    TEMP_B.rows = OBS_SIZE; TEMP_B.cols = 1;
    MatCalc(&Z, &TEMP_A, &TEMP_B, '-');   // TEMP_B = z - Hx

    // TEMP_A := K * TEMP_B  (state x 1)
    TEMP_A.rows = STATE_SIZE; TEMP_A.cols = 1;
    MatCalc(&K, &TEMP_B, &TEMP_A, '*');     // TEMP_A = K * (z - Hx)

    // OUT := PRE + TEMP_A  (state x 1)
    MatCalc(&PRE, &TEMP_A, &OUT, '+');     // OUT = PRE + TEMP_A

    // ---------- P = (I - K * H) * P ----------
    // TEMP_A := K * H  (state x state)
    TEMP_A.rows = STATE_SIZE; TEMP_A.cols = STATE_SIZE;
    MatCalc(&K, &H, &TEMP_A, '*');        // TEMP_A = K * H

    // TEMP_A := I - TEMP_A  (state x state)
    MatCalc(&I, &TEMP_A, &TEMP_A, '-');     // TEMP_A = I - K*H

    // P := TEMP_A * P  (state x state)
    MatCalc(&TEMP_A, &P, &P, '*');        // P = (I - K*H) * P

#if KALMAN_USE_SMOOTHER
    Smoother();
#endif
}

#if KALMAN_USE_SMOOTHER
void KalmanFilter::GetData(float* out_states) {
    for (int i = 0; i < STATE_SIZE; i++) {
        out_states[i] = smooth_output[i];
    }
}
#else
void KalmanFilter::GetData(float* out_states) {
    for (int i = 0; i < STATE_SIZE; i++) {
        out_states[i] = output[i];
    }
}
#endif

#if KALMAN_USE_SMOOTHER
// 固定αを使ったEMA/MAブレンドスムーザー
void KalmanFilter::Smoother() {
    float alpha = SMOOTHER_ALPHA;
    // clamp for safety
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        if (SMOOTHER_WINDOW_SIZE > 1) {
            // push into circular buffer for this state
            smooth_buffer[i][smooth_index % SMOOTHER_WINDOW_SIZE] = output[i];
            // compute moving average over available entries
            float sum = 0.0f;
            int count = (smooth_index < SMOOTHER_WINDOW_SIZE) ? (smooth_index + 1) : SMOOTHER_WINDOW_SIZE;
            for (int j = 0; j < count; j++) sum += smooth_buffer[i][j];
            float ma = sum / (float)count;
            if (smooth_index == 0) {
                smooth_output[i] = output[i];
            } else {
                float ema = alpha * output[i] + (1.0f - alpha) * smooth_output[i];
                // Blend MA and EMA: weight of MA grows with full window
                float w = (float)count / (float)SMOOTHER_WINDOW_SIZE;
                if (w > 1.0f) w = 1.0f;
                smooth_output[i] = w * ma + (1.0f - w) * ema;
            }
        } else {
            if (smooth_index == 0) {
                smooth_output[i] = output[i];
            } else {
                smooth_output[i] = alpha * output[i] + (1.0f - alpha) * smooth_output[i];
            }
        }
    // no per-state previous-output needed for fixed-alpha smoother
    }
    smooth_index++;
}
#endif


// Simple online noise estimation moved from Altitude; keeps EWMA of measurement and
// prediction variances and updates observation_noise and prediction_noise members.
void KalmanFilter::EstimateNoise(float meas) {
    const float EPS_NOISE = 1e-6f;
    // static locals hold internal noise estimation state (function-scoped, static lifetime)
    static float noise_meas_mean = 0.0f;
    static float noise_meas_var = 1.0f;
    static float noise_proc_mean = 0.0f;
    static float noise_proc_var = 0.1f;
    static float noise_prev_pred0 = 0.0f;

    // measurement mean/variance (EWMA)
    noise_meas_mean = (1.0f - NOISE_ALPHA) * noise_meas_mean + NOISE_ALPHA * meas;
    float v = meas - noise_meas_mean;
    noise_meas_var = (1.0f - NOISE_ALPHA) * noise_meas_var + NOISE_ALPHA * v * v;

    // prediction error proxy: use PRE[0] (prediction vector first element)
    float pred0 = 0.0f;
    if (PRE.data) {
        pred0 = PRE.data[0];
    }
    float pred_err = pred0 - noise_prev_pred0;
    noise_proc_mean = (1.0f - NOISE_ALPHA) * noise_proc_mean + NOISE_ALPHA * pred_err;
    float pv = pred_err - noise_proc_mean;
    noise_proc_var = (1.0f - NOISE_ALPHA) * noise_proc_var + NOISE_ALPHA * pv * pv;

    // clamp and reflect into Kalman matrices
    if (noise_meas_var > EPS_NOISE) {
        observation_noise = noise_meas_var;
    } else {
        observation_noise = EPS_NOISE;
    }
    if (noise_proc_var > EPS_NOISE) {
        prediction_noise = noise_proc_var;
    } else {
        prediction_noise = EPS_NOISE;
    }

    noise_prev_pred0 = pred0;
}




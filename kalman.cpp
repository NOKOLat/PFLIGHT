/*
 * kalman.cpp
 *
 *  Created on: Aug 12, 2025
 *      Author: takut
 */

#include "kalman.h"
#include <string.h>

void Kalman::Init(uint8_t state_size, uint8_t obs_size) {

    STATE_SIZE = state_size;
    OBS_SIZE = obs_size;

    // 各MATRIXメンバにdata/rows/colsをセット
    P.data = prediction_covariance;     P.rows = state_size;    P.cols = state_size;
    Q.data = prediction_noise_matrix;   Q.rows = state_size;    Q.cols = state_size;
    R.data = observation_noise_matrix;  R.rows = obs_size;      R.cols = obs_size;
    K.data = kalman_gain;               K.rows = state_size;    K.cols = obs_size;
    I.data = identity_matrix;           I.rows = state_size;    I.cols = state_size;
    F.data = system_matrix;             F.rows = state_size;    F.cols = state_size;
    H.data = observation_matrix;        H.rows = obs_size;      H.cols = state_size;
    Z.data = observation;               Z.rows = obs_size;      Z.cols = 1;
    PRE.data = prediction;              PRE.rows = state_size;  PRE.cols = 1;
    OUT.data = output;                  OUT.rows = state_size;  OUT.cols = 1;
    TEMP.data = temp;                   TEMP.rows = state_size; TEMP.cols = state_size;
    TEMP1.data = temp1;                 TEMP1.rows = state_size;TEMP1.cols = obs_size;
    TEMP2.data = temp2;                 TEMP2.rows = obs_size;  TEMP2.cols = state_size;
    TEMP3.data = temp3;                 TEMP3.rows = obs_size;  TEMP3.cols = obs_size;

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
    MatInit(&TEMP);
    MatInit(&TEMP1);
    MatInit(&TEMP2);
    MatInit(&TEMP3);

    // P, I を対角1で初期化
    for (int i = 0; i < state_size; i++) {
        prediction_covariance[i * state_size + i] = 1.0f;
        identity_matrix[i * state_size + i] = 1.0f;
    }

    is_initialized = true;
}

void Kalman::Update() {
    if (!is_initialized) {
        return;
    }

    // Q, R をスカラーから対角行列に反映
    MatInit(&Q);
    MatInit(&R);
    for (uint8_t i = 0; i < STATE_SIZE; ++i) {
        prediction_noise_matrix[i * STATE_SIZE + i] = prediction_noise;
    }
    for (uint8_t i = 0; i < OBS_SIZE; ++i) {
        observation_noise_matrix[i * OBS_SIZE + i] = observation_noise;
    }

    // TEMP2: F*P, TEMP: F^T, TEMP1: HP, TEMP3: O, TEMP: HT, TEMP3: O^-1, TEMP1: HX, TEMP3: RES

    // P = F * P * Ft + Q
    MatCalc(&F, &P, &TEMP2, '*');        // TEMP2 = F * P
    MatCalc(&F, nullptr, &TEMP, 't');    // TEMP = F^T
    MatCalc(&TEMP2, &TEMP, &P, '*');     // P = TEMP2 * F^T
    MatCalc(&P, &Q, &P, '+');            // P = P + Q

    // O = H * P * Ht + R
    MatCalc(&H, &P, &TEMP1, '*');        // TEMP1 = H * P
    MatCalc(&H, nullptr, &TEMP, 't');    // TEMP = H^T
    MatCalc(&TEMP1, &TEMP, &TEMP3, '*'); // TEMP3 = HP * H^T = O
    MatCalc(&TEMP3, &R, &TEMP3, '+');    // TEMP3 = O + R

    // K = P * H^T * O^-1
    MatCalc(&TEMP3, nullptr, &TEMP, 'i'); // TEMP = O^-1
    MatCalc(&P, &TEMP, &K, '*');          // K = P * H^T
    MatCalc(&K, &TEMP, &K, '*');          // K = K * O^-1

    // output = prediction + K * (observation - H * prediction)
    MatCalc(&H, &PRE, &TEMP1, '*');       // TEMP1 = H * prediction (1x1)
    MatCalc(&Z, &TEMP1, &TEMP3, '-');     // TEMP3 = z - Hx (1x1)
    MatCalc(&K, &TEMP3, &OUT, '*');       // OUT = K * (z-Hx)
    MatCalc(&PRE, &OUT, &PRE, '+');       // prediction = prediction + OUT

    // P = (I - K * H) * P
    MatCalc(&K, &H, &TEMP, '*');          // TEMP = K * H
    MatCalc(&I, &TEMP, &TEMP, '-');       // TEMP = I - K * H
    MatCalc(&TEMP, &P, &P, '*');          // P = TEMP * P
}

void Kalman::GetData(float* out_states) {
    if (!is_initialized || !out_states) return;
    for (int i = 0; i < STATE_SIZE; i++) {
        out_states[i] = prediction[i];
    }
}




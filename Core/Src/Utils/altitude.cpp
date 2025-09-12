/*
 * altitude.cpp
 *
 *  Created on: Aug 12, 2025
 *      Author: takut
 */

#include "Utils/altitude.h"
#include <math.h>
#include "stdio.h"

#define ALTITUDE_PID_TIME 0.010f
#define VELOCITY_PID_TIME 0.0025f
#define G_CONST 9.81f

Altitude altitude;

void Altitude::Init() {
    kalman.Init(2,1);
    // 初期状態
    // 内部は m 単位
    estimated_altitude = 0.0f; // m
    estimated_velocity = 0.0f; // m/s
    estimated_accel = 0.0f;    // m/s^2
    reference_pressure = 101325.0f;

}

void Altitude::Update(float pressure_Pa, float accel[3], float angle[3], float dt) {

    // -- 重力補正 --
    const float PI_F = 3.14159265358979323846f;
    float roll = angle[0] * PI_F / 180.0f;
    float pitch = angle[1] * PI_F / 180.0f;
    float gravity_z = cosf(roll) * cosf(pitch);
    // accel は m/s^2、内部も m/s^2 を使用
    float accel_z_m_s2 = accel[2] - (gravity_z * G_CONST);
    estimated_accel = accel_z_m_s2; // m/s^2

    // デッドバンド処理: キャリブで決めた accel_deadband を使用（m/s^2 単位）
    if (estimated_accel > -accel_deadband && estimated_accel < accel_deadband) {
        estimated_accel = 0.0f;
    }

    // -- システム/観測行列 & 予測-
    kalman.system_matrix[0] = 1.0f; kalman.system_matrix[1] = dt;
    kalman.system_matrix[2] = 0.0f; kalman.system_matrix[3] = 1.0f;
    kalman.observation_matrix[0] = 1.0f; kalman.observation_matrix[1] = 0.0f;

    // Kalman は内部単位 m / m/s / m/s^2 を用いる
    kalman.prediction[0] = estimated_altitude + estimated_velocity * dt + 0.5f * estimated_accel * dt * dt;
    kalman.prediction[1] = estimated_velocity + estimated_accel * dt;

    // 気圧を高度に変換
    if (!(pressure_Pa > 0.0f) || isnan(pressure_Pa)) {
        // センサ不在時は予測のみ
        estimated_altitude = kalman.prediction[0];
        estimated_velocity = kalman.prediction[1];
        return;
    }
    // 気圧から得られる高度は m 単位なのでそのまま観測値とする
    float meas = 44330.0f * (1.0f - powf(pressure_Pa / reference_pressure, 0.1903f));
    // Delegate noise estimation to KalmanFilter instance
    kalman.EstimateNoise(meas);

    // 観測を与えて更新 (m 単位)
    kalman.observation[0] = meas;
    kalman.Update();

    float out[2];
    kalman.GetData(out);
    estimated_altitude = out[0]; // m
    estimated_velocity = out[1]; // m/s
    return;
}


void Altitude::Calibration(float pressure_Pa, float observed_accel){
    // calibration 呼び出し回数（メンバに移動）
    calib_count++;

    // reference_pressure: 逐次的平均
    if (calib_count == 1) {
        reference_pressure = pressure_Pa;
    } else {
        reference_pressure += (pressure_Pa - reference_pressure) / (float)calib_count;
    }

    // Welford の逐次平均/分散更新 (accel_calib_mean, accel_calib_M2 はメンバ変数)
    // observed_accel は m/s^2。内部のキャリブも m/s^2 で保持
    float delta = observed_accel - accel_calib_mean;
    accel_calib_mean += delta / (float)calib_count;
    float delta2 = observed_accel - accel_calib_mean;
    accel_calib_M2 += delta * delta2;

    // deadband は静止時揺らぎの 3σ を下限 0.01 m/s^2 として設定
    float sigma = 0.0f;
    if (calib_count > 1) sigma = sqrtf(accel_calib_M2 / (float)(calib_count - 1));
    if (3.0f * sigma > 0.01f) {
        accel_deadband = 3.0f * sigma;
    } else {
        accel_deadband = 0.01f;
    }

    //デバッグ用出力（任意）
    printf("Calib #%u: refP=%.2f, accel_mean=%.4f, deadband=%.4f\n", (unsigned)calib_count, reference_pressure, accel_calib_mean, accel_deadband);
}

// Altitude::EstimateNoise removed — noise estimation moved to KalmanFilter

void Altitude::GetData(float *out) {
    // 内部は m 単位なのでそのまま出力。速度は m/s, accelは m/s^2
    out[0] = estimated_altitude;
    out[1] = estimated_velocity;
    out[2] = estimated_accel;
}

void Altitude::Reset() {
    // PIDs removed; nothing to reset for them
    kalman.Init(2,1);
    estimated_altitude = 0.0f; // m
    estimated_velocity = 0.0f; // m/s
    // clear calibration state
    calib_count = 0u;
    accel_calib_mean = 0.0f;
    accel_calib_M2 = 0.0f;
    accel_deadband = 0.01f; // m/s^2
}





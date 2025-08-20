/*
 * altitude.cpp
 *
 *  Created on: Aug 12, 2025
 *      Author: takut
 */

#include "altitude.h"
#include <math.h>

#define ALTITUDE_PID_TIME 0.010f
#define VELOCITY_PID_TIME 0.0025f

Altitude altitude;

void Altitude::Init() {
    // カルマンフィルタ初期化
    kalman.Init(2,1);

    // PID制御器初期化
    altitude_pid.GainSet(2.0f, 0.1f, 0.5f);
    altitude_pid.TimeSet(ALTITUDE_PID_TIME);
    altitude_pid.LimitSet(20.0f, 50.0f);

    velocity_pid.GainSet(1.5f, 0.0f, 0.01f);
    velocity_pid.TimeSet(VELOCITY_PID_TIME);
    velocity_pid.LimitSet(0.0f, 30.0f);

    acceleration_pid.GainSet(0.8f, 0.0f, 0.001f);
    acceleration_pid.TimeSet(VELOCITY_PID_TIME);
    acceleration_pid.LimitSet(0.0f, 20.0f);

    // 初期状態設定
    target_altitude = 0.0f;
    target_velocity = 0.0f;
    estimated_altitude = 0.0f;
    estimated_velocity = 0.0f;
    throttle_correction = 0.0f;
    reference_pressure = 1013.25f;

    is_initialized = true;
}

void Altitude::Update(float pressure_hPa, float accel[3], float angle[3], float dt) {
    if (!is_initialized) {
        return;
    }
    // 加速度補正（重力成分除去）
    estimated_accel = CorrectAcceleration(accel, angle);

    // カルマンフィルタのパラメータ設定
    // システム行列（状態遷移モデル）
    kalman.system_matrix[0] = 1.0f; kalman.system_matrix[1] = dt;   // [1, dt]
    kalman.system_matrix[2] = 0.0f; kalman.system_matrix[3] = 1.0f; // [0, 1]

    // 観測行列（高度のみ観測）
    kalman.observation_matrix[0] = 1.0f; kalman.observation_matrix[1] = 0.0f; // [1, 0]

    // 予測値設定（運動学モデルによる予測）
    kalman.prediction[0] = estimated_altitude + estimated_velocity * dt + 0.5f * estimated_accel * dt * dt;
    kalman.prediction[1] = estimated_velocity + estimated_accel * dt;

    // 予測ノイズ
    kalman.prediction_noise = 0.01f;
    // 観測ノイズ
    kalman.observation_noise = 1.0f;

    // 気圧センサデータが有効か判定
    if (pressure_hPa > 0.0f && !isnan(pressure_hPa)) {
        float altitude_measurement = PressureToAltitude(pressure_hPa);
        kalman.observation[0] = altitude_measurement;

        // カルマンフィルタ更新
        kalman.Update();

        // 推定結果取得
        float result[2];
        kalman.GetData(result);
        estimated_altitude = result[0];  // 高度
        estimated_velocity = result[1];  // 速度
    } else {
        // 気圧センサが無い場合は予測のみ
        estimated_altitude = kalman.prediction[0];
        estimated_velocity = kalman.prediction[1];
    }
}

void Altitude::Calc() {
    if (!is_initialized) return;
    loop_count++;
    // 100Hz 高度制御
    if(loop_count % 4 == 0) {
        altitude_pid.Calc(estimated_altitude, target_altitude);
        target_velocity = altitude_pid.GetData();
    }
    // 400Hz 速度制御
    velocity_pid.Calc(estimated_velocity, target_velocity);
    float target_acceleration = velocity_pid.GetData();
    // 400Hz 加速度制御
    acceleration_pid.Calc(estimated_accel, target_acceleration);
    throttle_correction = acceleration_pid.GetData();
}

void Altitude::Reset() {
    altitude_pid.Reset();
    velocity_pid.Reset();
    acceleration_pid.Reset();
    
    // カルマンフィルタリセット
    kalman.Init(2,1);
    
    target_altitude = 0.0f;
    target_velocity = 0.0f;
    throttle_correction = 0.0f;
    estimated_altitude = 0.0f;
    estimated_velocity = 0.0f;
}

float Altitude::PressureToAltitude(float pressure_hPa) const {
    return 44330.0f * (1.0f - powf(pressure_hPa / reference_pressure, 0.1903f));
}

float Altitude::CorrectAcceleration(float accel[3], float angle[3]) const {
    // Madgwick姿勢推定結果を使用した重力補正
    const float PI_F = 3.14159265358979323846f;
    float roll = angle[0] * PI_F / 180.0f;
    float pitch = angle[1] * PI_F / 180.0f;
    
    // 重力ベクトルを機体座標系に変換
    float gravity_z = cosf(roll) * cosf(pitch);
    
    // Z軸加速度から重力成分を除去（9.81f = G定数）
    return accel[2] - (gravity_z * 9.81f);
}

void Altitude::SetTarget(float altitude) {
    target_altitude = altitude;
}

float Altitude::getData() const {
    return throttle_correction;
}

float Altitude::getAltitude() const {
    return estimated_altitude;
}

float Altitude::getVelocity() const {
    return estimated_velocity;
}





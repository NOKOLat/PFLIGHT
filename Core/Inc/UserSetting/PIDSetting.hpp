#ifndef PIDSETTING_HPP
#define PIDSETTING_HPP

#include "PID/PID.h"

// ゲイン用構造体
struct PIDGain {
    float kp;
    float ki;
    float kd;
};

// 制限値用構造体（dtを除外）
struct PIDLimit {
    float i_max;
    float d_max;
};

// dt専用構造体（または定数）
struct PIDDt {
    float dt;
};

// 制御周期
// 変更する場合はTIMの割り込み設定や実行速度の測定を行うこと
// わからないなら、400から動かさないこと
constexpr float sampleFrequency = 400.0f;

// PIDインスタンス生成用の設定
namespace UserSetting {

    // 角度PID設定
	constexpr PIDGain angle_pitch_gain = { 2.5, 0.01, 0.10 };
	constexpr PIDLimit angle_pitch_limit = { 20.0, 50.0 };
	constexpr PIDDt angle_pitch_dt = { 4.0f / sampleFrequency};

	constexpr PIDGain angle_roll_gain = { 2.5, 0.01, 0.10 };
	constexpr PIDLimit angle_roll_limit = { 20.0, 50.0 };
	constexpr PIDDt angle_roll_dt = { 4.0f / sampleFrequency};

	// 角速度PID設定
	constexpr PIDGain rate_pitch_gain = { 1.5, 0.0, 0.05 };
	constexpr PIDLimit rate_pitch_limit = { 0.0, 100.0};
	constexpr PIDDt rate_pitch_dt = { 1.0f / sampleFrequency};

	constexpr PIDGain rate_roll_gain = { 1.5, 0.0, 0.05 };
	constexpr PIDLimit rate_roll_limit = { 0.0, 100.0 };
	constexpr PIDDt rate_roll_dt = { 1.0f / sampleFrequency};

	constexpr PIDGain rate_yaw_gain = { 6, 0.0, 0.2 };
	constexpr PIDLimit rate_yaw_limit = { 0.0, 40.0};
	constexpr PIDDt rate_yaw_dt = { 1.0f / sampleFrequency};

    //Madgwick用のサンプル周波数
    constexpr float MadgwickSampleFreq = sampleFrequency;
}

#endif // PIDSETTING_HPP

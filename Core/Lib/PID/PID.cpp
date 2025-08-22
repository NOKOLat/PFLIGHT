/*
 * PID.cpp
 *
 *  Created on: Feb 19, 2025
 *      Author: Sezakiaoi
 */

#include "PID.h"
#include <cmath>

/*
 * デフォルトコンストラクタ
 *
 * ゲインや制御周期などはSetGain()やSetTime()で設定してください
 */
PID::PID(){

}

/*
 * コンストラクタ
 *
 * ゲイン、制御周期、積分項と微分項の制限値を設定します
 */
PID::PID(float p_gain, float i_gain, float d_gain, float d_max, float i_max, float time){

	setGain(p_gain, i_gain, d_gain);
	setTime(time);
	setLimit(i_max, d_max);
}

/*
 * コンストラクタ
 *
 * ゲイン、制御周期のみを設定します
 * 積分項と微分項の制限値はLimitSet()で設定してください
 */
PID::PID(float p_gain, float i_gain, float d_gain, float time) {

	setGain(p_gain, i_gain, d_gain);
	setTime(time);
	setLimit(1000.0f, 1000.0f);
}

/*
 * ゲインの設定
 *
 * PIDのゲインを設定します
 * 
 * 有効な正の値であるかを確認し
 * 満たさない場合は0に設定します
 */
void PID::setGain(float p_gain, float i_gain, float d_gain){

	if (std::isfinite(p_gain) && p_gain > 0.0f) {

		this->p_gain = p_gain;
	} 
	else {

		this->p_gain = 0.0f;
	}

	if (std::isfinite(i_gain) && i_gain > 0.0f) {

		this->i_gain = i_gain;
	} 
	else {
		this->i_gain = 0.0f;
	}

	if (std::isfinite(d_gain) && d_gain > 0.0f) {

		this->d_gain = d_gain;
	} 
	else {

		this->d_gain = 0.0f;
	}
}

/*
 * 制御周期の設定
 *
 * 有効な正の値であるかを確認し
 * 満たさない場合は最小値(1e-6)に設定します
 */
void PID::setTime(float time){

	if (!std::isfinite(time) || time <= time_min) {

		this->time = time_min;
	} 
	else {

		this->time = time;
	}

}

/*
 * 制御周期の設定
 *
 * 有効な正の値であるかを確認し
 * 満たさない場合は最小値(1e-6)に設定します
 */

void PID::setLimit(float i_max, float d_max){

	if (!std::isfinite(i_max) || i_max < 0.0f) {

		this->i_max = 0.0f;
	} 
	else {

		this->i_max = i_max;
	}

	if (!std::isfinite(d_max) || d_max < 0.0f) {

		this->d_max = 0.0f;
	} 
	else {

		this->d_max = d_max;
	}

}

/*
 * 制御計算
 * 
 * 通常のPID制御計算を行います
 * 積分計算はシンプソン公式を用いて行います
 */
void PID::calc(float target, float process_value){

	// P制御の計算
	float error = target - process_value;

	// I制御の計算(3点でのシンプソン公式)
	// サンプルカウントが2未満の間は台形法
	if (sample_count < 2) {

		// 初回・2回目までは台形近似
		integral += (error + pre_error) * 0.5f * time;
		sample_count++;
	} 
	else {

		// 3サンプル目以降: 直近3点に対するシンプソン公式
		// integral += (time/3) * (e_{k-2} + 4*e_{k-1} + e_k)
		integral += (time / 3.0f) * (pre_error2 + 4.0f * pre_error + error);
	}
	
	// 積分項の制限
	if (std::fabs(integral) > i_max) {

		integral = std::copysign(i_max, integral);
	}

	float delta = (error - pre_error) / time;
	
	// 微分項の制限
	if (std::fabs(delta) > d_max) {

		delta = std::copysign(d_max, delta);
	}

	control = error * p_gain + integral * i_gain + delta * d_gain;

	// 誤差履歴更新
	pre_error2 = pre_error;
	pre_error = error;
}

float PID::getData(){

	return control;
}

void PID::getData(float* out){

	if (out) {
		*out = control;
	}
}

void PID::reset(){

	// ゲイン(p/i/d)、周期(time)、上限(i_max/d_max)は維持
	integral   = 0.0f;
	pre_error  = 0.0f;
	pre_error2 = 0.0f;
	control    = 0.0f;
	sample_count = 0;
}

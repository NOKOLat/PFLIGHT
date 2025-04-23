/*
 * PID.cpp
 *
 *  Created on: Feb 19, 2025
 *      Author: Sezakiaoi
 */

#include <PID.h>

/**
 * @brief PIDのコンストラクタ
 *
 * @param[in] gain_p Pゲイン
 * @param[in] gain_i Iゲイン
 * @param[in] gain_d Dゲイン
 *
 * @return None
 */
PID::PID(float gain_p, float gain_i, float gain_d){

	this->gain_p = gain_p;
	this->gain_i = gain_i;
	this->gain_d = gain_d;
}

/**
 * @brief PIDのデータをセット
 *
 * @param[in] target   目標値
 * @param[in] current    現在値
 * @param[in] time   実行間隔（微積分に使用）
 *
 * @return None
 */
void PID::SetData(float target, float current, float time){

	this->target = target;
	this->current = current;
	this->time = time;
}

/**
 * @brief PIDの計算
 * 
 * @return None
 */
void PID::Calc(){

	float error = 0.0;
	float diff = 0.0;


	error = target - current;
	integral += (error + pre_error) / 2.0 * time;
	diff = (pre_error - error) / time;

	pre_error = error;

	control = error * gain_p + integral * gain_i + diff * gain_d;
}

/**
 * @brief PIDの値を取得
 *
 * @return PIDの値
 */
float PID::GetData(){

	return control;
}

/**
 * @brief PIDの初期化
 *
 * @return None
 */
void PID::Reset(){

	gain_p = 0;
	gain_i = 0;
	gain_d = 0;
	integral  = 0;
	pre_error = 0;
}

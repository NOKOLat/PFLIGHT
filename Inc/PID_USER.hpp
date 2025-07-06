#ifndef INC_PID_USER_HPP_
#define INC_PID_USER_HPP_

#include <cstdint>
#include "pid.h"

PID angle_pitch;
PID angle_roll;

PID rate_pitch;
PID rate_roll;
PID rate_yaw;


void PidSetup(){

//	Pで偏差に対する応答量の調節
//	偏差が小さくなると目標値手前で収束する→Iで引き上げる
//	加速しすぎないように微分値Dで抑える

//	Pで応答上げて
//	Dで抑えて
//	目標値とのズレをIで合わせる

	//角度制御(100hz)
	angle_pitch.Setup(0.85, 0, 0, 0.010);
	angle_roll.Setup(0.85, 0, 0, 0.010);

	//角速度制御(400hz)
	rate_pitch.Setup(1, 0, 0, 0.0025);
	rate_roll.Setup(1, 0, 0, 0.0025);
	rate_yaw.Setup(1, 0, 0.1, 0.0025);
}

void AnglePIDCalc(float current[2], float target[2]){

	angle_pitch.Calc(current[0], target[0]);
	angle_roll.Calc(current[1], target[1]);
}

void RatePIDCalc(float current[3], float target[3]){

	rate_pitch.Calc(current[0], target[0]);
	rate_roll.Calc(current[1], target[1]);
	rate_yaw.Calc(current[2], target[2]);
}

void AnglePIDGetData(float target_rate[3]){

	target_rate[0] = angle_pitch.GetData();
	target_rate[1] = angle_roll.GetData();
}

void RatePIDGetData(float rate[3]){

	rate[0] = rate_pitch.GetData();
	rate[1] = rate_roll.GetData();
	rate[2] = rate_yaw.GetData();
}

void PidReset(){

	angle_pitch.Reset();
	angle_roll.Reset();

	rate_pitch.Reset();
	rate_roll.Reset();
	rate_yaw.Reset();
}


#endif

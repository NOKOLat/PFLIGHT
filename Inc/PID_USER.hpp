#ifndef INC_PID_USER_HPP_
#define INC_PID_USER_HPP_

#include <cstdint>
#include "pid.h"

PID angle_pitch;
PID angle_roll;

PID rate_pitch;
PID rate_roll;
PID rate_yaw;

AnglePID angle;
RatePID rate;


void PidSetup(){

    //角度制御(100hz)
    angle_pitch.Setup(angle.pitch[0], angle.pitch[1], angle.pitch[2], angle.time);
    angle_roll.Setup(angle.roll[0], angle.roll[1], angle.roll[2], angle.time);

    //角速度制御(400hz)
    rate_pitch.Setup(rate.pitch[0], rate.pitch[1], rate.pitch[2], rate.time);
    rate_roll.Setup(rate.roll[0], rate.roll[1], rate.roll[2], rate.time);
    rate_yaw.Setup(rate.yaw[0], rate.yaw[1], rate.yaw[2], rate.time);
}

void AnglePIDCalc(float current[3], float target[3]){

	angle_pitch.Calc(current[0], target[0]);
	angle_roll.Calc(current[1], target[1]);
}

void RatePIDCalc(float current[3], float target[3]){

	rate_pitch.Calc(current[0], target[0]);
	rate_roll.Calc(current[1], target[1]);
	rate_yaw.Calc(current[2], target[2]);
}

void AnglePIDGetData(float angle[3]){

	angle[0] = angle_pitch.GetData();
	angle[1] = angle_roll.GetData();
}

void RatePIDGetData(float rate[3]){

	rate[0] = rate_pitch.GetData();
	rate[1] = rate_roll.GetData();
	rate[2] = rate_yaw.GetData();
}


#endif

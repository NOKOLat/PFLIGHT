#ifndef INC_PID_USER_HPP_
#define INC_PID_USER_HPP_

#include <cstdint>
#include "pid.h"

PID pitch;
PID roll;
PID yaw;

void PidSetup(){

	pitch.Setup(2.5, 0, 0, 0);
	roll.Setup(2.5, 0, 0, 0);
	yaw.Setup(0.5, 0, 0, 0);

}

void PidCalc(float now[3], float target[3]){

	pitch.Calc(now[0], target[0]);
	roll.Calc(now[1], target[1]);
	yaw.Calc(now[2], target[2]);
}

void PidGetData(float control[3]){

	control[0] = pitch.GetData();
	control[1] = roll.GetData();
	control[2] = yaw.GetData();
}


#endif

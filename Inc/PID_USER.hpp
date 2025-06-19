#ifndef INC_PID_USER_HPP_
#define INC_PID_USER_HPP_

#include <cstdint>
#include "pid.h"

PID angle_pitch;
PID angle_roll;

PID speed_pitch;
PID speed_roll;
PID speed_yaw;


void PidSetup(){

	//角度制御(100hz)
	angle_pitch.Setup(4.2, 0.0, 0.0, 0.0010);
	angle_roll.Setup(4.2, 0.0, 0.0, 0.0010);

	//角速度制御(400hz)
	speed_pitch.Setup(0.85, 0.1, 0.3, 0.00025);
	speed_roll.Setup(0.85, 0.1, 0.3, 0.00025);
	speed_yaw.Setup(0.85, 0.1, 0.3, 0.00025);
}

void PidAngleCalc(float current[3], float target[3]){

	angle_pitch.Calc(current[0], target[0]);
	angle_roll.Calc(current[1], target[1]);
}

void PidSpeedCalc(float current[3], float target[3]){

	speed_pitch.Calc(current[0], target[0]);
	speed_roll.Calc(current[1], target[1]);
	speed_yaw.Calc(current[2], target[2]);
}

void PIDGetAngle(float angle[3]){

	angle[0] = angle_pitch.GetData();
	angle[1] = angle_roll.GetData();
}

void PIDGetSpeed(float speed[3]){

	speed[0] = speed_pitch.GetData();
	speed[1] = speed_roll.GetData();
	speed[2] = speed_yaw.GetData();
}


#endif

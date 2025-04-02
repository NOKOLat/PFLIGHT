/*
 * PID.cpp
 *
 *  Created on: Feb 19, 2025
 *      Author: Sezakiaoi
 */

#include <PID.h>


void PID::Setup(float Input_Gain_P, float Input_Gain_I, float Input_Gain_D, float Input_Goal){

	Gain_P = Input_Gain_P;
	Gain_I = Input_Gain_I;
	Gain_D = Input_Gain_D;
	Goal   = Input_Goal;
}

float PID::Calc(float Angle, float Angular_Velocity){

	float Error = Goal - Angle;
	float Delta = Angular_Velocity;
	float Integral = (Error - Pre_Error) * Time;

	return Error * Gain_P + Delta * Gain_I + Integral * Gain_D;
}

void PID::Reset(){

	Gain_P = 0;
	Gain_I = 0;
	Gain_D = 0;
	Goal   = 0;
	Pre_Error = 0;
}

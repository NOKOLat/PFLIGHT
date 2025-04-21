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

void PID::Calc(float Angle, float Goal){

	float Error = Goal - Angle;
	integral += (Error + Pre_Error) / 2 * Time;
	float Delta = (Pre_Error - Error) / Time;

	control = Error * Gain_P + integral * Gain_I + Delta * Gain_D;

	Pre_Error = Error;
}

float PID::GetData(){

	return control;
}

void PID::Reset(){

	Gain_P = 0;
	Gain_I = 0;
	Gain_D = 0;
	Goal   = 0;
	Pre_Error = 0;
}

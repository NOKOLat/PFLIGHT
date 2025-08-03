/*
 * PID.cpp
 *
 *  Created on: Feb 19, 2025
 *      Author: Sezakiaoi
 */

#include <PID.h>


void PID::Setup(float Input_Gain_P, float Input_Gain_I, float Input_Gain_D, float time){

	Gain_P = Input_Gain_P;
	Gain_I = Input_Gain_I;
	Gain_D = Input_Gain_D;
	this->Time = time;

}

void PID::GainSet(float kp, float ki, float kd){

	Gain_P = kp;
	Gain_I = ki;
	Gain_D = kd;

}

void PID::TimeSet(float time){

	this->Time = time;

}

void PID::LimitSet(float i_max, float d_max){

	I_Max = i_max;
	D_Max = d_max;

}

void PID::Calc(float Angle, float Goal){

	float Error = Goal - Angle;
	integral += (Error + Pre_Error) / 2 * Time;
	
	// 積分項の制限
	if(integral > I_Max){
		
		integral = I_Max;
	}
	else if(integral < -I_Max){

		integral = -I_Max;
	}

	float Delta = (Pre_Error - Error) / Time;
	
	// 微分項の制限
	if(Delta > D_Max){

		Delta = D_Max;
	}
	else if(Delta < -D_Max){

		Delta = -D_Max;
	}

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
	integral = 0; 
}

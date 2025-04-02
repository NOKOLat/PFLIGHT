/*
 * PID.h
 *
 *  Created on: Feb 19, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_PID_H_
#define INC_PID_H_

class PID {

	public:

		void Setup(float Input_Gain_P, float Input_Gain_I, float Input_Gain_D, float Input_Goal);
		float Calc(float Target, float Angle);
		void Reset();

	private:

		float Gain_P    = 0.0;
		float Gain_I    = 0.0;
		float Gain_D    = 0.0;
		float Goal      = 0.0;
		float Pre_Error = 0.0;
		float Time      = 1.0 / 100.0;
};

#endif /* INC_PID_H_ */

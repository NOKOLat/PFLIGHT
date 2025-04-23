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

		PID(float gain_p, float gain_i, float gain_d);
		void SetData(float goal, float now_value, float time);
		void Calc();
		float GetData();
		void Reset();

	private:

		float current;
		float target;
		float time;

		float gain_p    = 0.0;
		float gain_i    = 0.0;
		float gain_d  	= 0.0;
		float pre_error = 0.0;
		float integral  = 0.0;

		float control = 0.0;
};

#endif /* INC_PID_H_ */

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

		void Setup(float Input_Gain_P, float Input_Gain_I, float Input_Gain_D, float time);
		void GainSet(float kp, float ki, float kd);
		void TimeSet(float time);
		void LimitSet(float i_max, float d_max);
		void Calc(float Target, float Angle);
		float GetData();
		void Reset();

	private:

		float Gain_P    = 0.0;
		float Gain_I    = 0.0;
		float Gain_D    = 0.0;
		float Goal      = 0.0;
		float Pre_Error = 0.0;
		float integral  = 0.0;
		float Time      = 0.000;
		float control   = 0.0;
		float I_Max     = 1000.0;  // 積分項の制限値
		float D_Max     = 1000.0;  // 微分項の制限値
};

#endif /* INC_PID_H_ */

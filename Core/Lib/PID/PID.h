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

		PID();
		PID(float p_gain, float i_gain, float d_gain, float time);
		PID(float p_gain, float i_gain, float d_gain, float d_max, float i_max, float time);
		void setGain(float p_gain, float i_gain, float d_gain);
		void setTime(float time);
		void setLimit(float i_max, float d_max);
		void calc(float target, float process_value);
		float getData();
		// ポインタを渡して結果を書き込むオーバーロード
		void getData(float* out);
		void reset();

	private:

		float p_gain    = 0.0f;
		float i_gain    = 0.0f;
		float d_gain    = 0.0f;	
		float i_max     = 1000.0f;  // 積分項の制限値
		float d_max     = 1000.0f;  // 微分項の制限値
		float pre_error = 0.0f;
		float pre_error2 = 0.0f;   // 2サンプル前の誤差（シンプソン補正用）
		float integral  = 0.0f;
		float time      = 0.0f;
		float control   = 0.0f;
		// ゼロ除算回避用の最小周期（調整可能にする場合はsetter追加を検討）
		float time_min  = 1e-6f;
		int   sample_count = 0;    // 取得済みサンプル数（シンプソン適用判定用）
};

#endif /* INC_PID_H_ */

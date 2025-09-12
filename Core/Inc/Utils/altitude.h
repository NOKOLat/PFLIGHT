/*
 * altitude.h
 *
 *  Created on: Aug 12, 2025
 *      Author: takut
 */

#ifndef INC_ALTITUDE_H_
#define INC_ALTITUDE_H_

#include "KalmanFilter/kalman_filter.hpp"

// 出力単位: 内部は m 単位で統一しています

class Altitude {

    public:

        void Init();
    void Update(float pressure_Pa, float accel[3], float angle[3], float dt);
    void Calibration(float pressure_Pa, float observed_accel);
        void Reset();
        void GetData(float *out);

    private:
    
        KalmanFilter kalman;
    // control targets and throttle removed (unused)
    // 内部単位を m / m/s / m/s^2 に統一
    float estimated_altitude = 0.0f; // m
    float estimated_velocity = 0.0f; // m/s
    float estimated_accel = 0.0f;    // m/s^2
        float reference_pressure = 0.0f;
        uint32_t calib_count = 0u; 

        // altitude offset applied to ensure non-negative reported altitude.
        // When estimated altitude goes below 0 the offset is increased
        // so reported altitude becomes 0 while preserving internal state.
        float altitude_offset = 0.0f; // m

        // calibration for accel: running mean and M2 for Welford variance
        float accel_calib_mean = 0.0f;
        float accel_calib_M2 = 0.0f;
    float accel_deadband = 0.01f; // default until calibrated (m/s^2)
    // noise estimation state delegated to KalmanFilter
            // キャリブレーション用（最初の N 回の気圧を平均してその時点を 0m とする）
        
};

#ifdef __cplusplus
extern Altitude altitude;
#endif

#endif /* INC_ALTITUDE_H_ */

/*
 * altitude.h
 *
 *  Created on: Aug 12, 2025
 *      Author: takut
 */

#ifndef INC_ALTITUDE_H_
#define INC_ALTITUDE_H_

#include "kalman.h"
#include "PID.h"

class Altitude {

    public:

        void Init();
    void Update(float pressure_hPa, float accel[3], float angle[3], float dt);
        void Calc();
        void Reset();
        void SetTarget(float altitude);
        float getData() const; // 高度に応じたスロットル値
        float getAltitude() const;
        float getVelocity() const;

    private:
    
        Kalman kalman;
        PID altitude_pid;
        PID velocity_pid;
        PID acceleration_pid;
        float target_altitude = 0.0f;
        float target_velocity = 0.0f;
        float throttle_correction = 0.0f;
        float estimated_altitude = 0.0f;
        float estimated_velocity = 0.0f;
        float estimated_accel = 0.0f;
        float reference_pressure = 1013.25f;
        bool is_initialized = false;
    uint32_t loop_count = 0u;
        float PressureToAltitude(float pressure_hPa) const;
        float CorrectAcceleration(float accel[3], float angle[3]) const;
};

#endif /* INC_ALTITUDE_H_ */

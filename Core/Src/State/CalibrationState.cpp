#include "State/Headers/FlightStates.h"

#ifndef CALIBRATION_STATE_CPP
#define CALIBRATION_STATE_CPP

void CalibrationState::enter(FlightManager& manager) {

    // センサー単体のキャリブレーション

    // IMUのキャリブレーション
    manager.imuUtil->calibration(1000);

    // 気圧センサーのキャリブレーション

}
void CalibrationState::update(FlightManager& manager) {

    // 6軸データの取得
    manager.imuUtil->getData(calibrated_accel, calibrated_gyro);
    if(calibration_count %4 == 0){
        
        // 気圧センサーのデータ取得
        float temperature = 0.0f;
        if (manager.dps368) {
            // DPS368 returns 0 on success and fills pressure (Pa) and temperature (C)
            if (manager.dps368->getData(&pressure, &temperature) != 0) {
                // read failed, keep previous or zero
                pressure = 0.0f;
            }
        } else {
            pressure = 0.0f; // 仮実装: センサ非搭載時は 0
        }

        // 高度キャリブレーションの計算
        // 観測加速度は IMU から取得した z 成分 (m/s^2)
        float observed_accel = calibrated_accel[2];

        // Altitude モジュールの逐次キャリブレーション呼び出し
        altitude.Calibration(pressure, observed_accel);
        // データ取得と計算が成功したらカウントを進める
    }
    calibration_count ++;

    // キャリブレーション完了後、PreFlightへ遷移
    if (calibration_count >= 400) {

        manager.changeState(std::make_unique<PreFlightState>());
        return;
    }
}

#endif // CALIBRATION_STATE_CPP
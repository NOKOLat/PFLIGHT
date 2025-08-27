#include "State/Headers/FlightStates.h"

#ifndef CALIBRATION_STATE_CPP
#define CALIBRATION_STATE_CPP

void CalibrationState::enter(FlightManager& manager) {

    // センサー単体のキャリブレーション

    // IMU
    manager.imuUtil->calibration(1000);

    // 気圧センサー

}
void CalibrationState::update(FlightManager& manager) {

    // 6軸データの取得
    manager.imuUtil->getData(calibrated_accel, calibrated_gyro);

    // 気圧センサーのデータ取得
    pressure = 0.0;//仮実装

    // 高度キャリブレーションの計算

    // データ取得と計算が成功したらカウントを進める
    calibration_count++;

    // キャリブレーション完了後、PreFlightへ遷移
    if (calibration_count >= 100) {

        manager.changeState(std::make_unique<PreFlightState>());
    }
}

#endif // CALIBRATION_STATE_CPP
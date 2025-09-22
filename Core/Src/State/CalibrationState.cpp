#include "State/Headers/FlightStates.h"

#ifndef CALIBRATION_STATE_CPP
#define CALIBRATION_STATE_CPP

void CalibrationState::enter(FlightManager& manager) {

    // センサー単体のキャリブレーション

    // IMUのキャリブレーション
    manager.imuUtil->Calibration(1000);
    calibration_count = 0;

    // 気圧センサーのキャリブレーション

}
void CalibrationState::update(FlightManager& manager) {
    static float sum_accel = 0.0f;
    
    float average_accel = 0.0f;
    
    std::array<float, 3> accel = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyro = {0.0f, 0.0f, 0.0f}; 
    
    float pressure = 0.0f;

    // 6軸データの取得
    manager.imuUtil->GetData(accel, gyro);
    sum_accel += accel[2];

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
        

        average_accel = sum_accel / 8.0f;
        sum_accel = 0.0f;
        // Altitude モジュールの逐次キャリブレーション呼び出し
        altitude.Calibration(pressure, average_accel);
        // データ取得と計算が成功したらカウントを進める
    }
    calibration_count ++;

    //キャリブレーション完了後、PreFlightへ遷移
    if (calibration_count >= 800) {

        //黄LEDをつける
        manager.yellow_led.Set(PinState::on);

        manager.changeState(std::make_unique<PreFlightState>());
        return;
    }
}

#endif // CALIBRATION_STATE_CPP

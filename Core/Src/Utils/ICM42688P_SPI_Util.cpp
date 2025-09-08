#include "Utils/ICM42688P_SPI_Util.hpp"

// ICM42688Pクラスに引数を渡す
ICM42688P_SPI_Util::ICM42688P_SPI_Util(SPI_HandleTypeDef* hspi, GPIO_TypeDef* gpio_port, uint32_t gpio_pin)
    : icm(hspi, gpio_port, gpio_pin) {}

// センサーの設定
uint8_t ICM42688P_SPI_Util::init() {

    if (icm.Connection() == 1) {

        return 1;
    }

    icm.AccelConfig(icm.ACCEL_Mode::LowNoize, icm.ACCEL_SCALE::SCALE02g, icm.ACCEL_ODR::ODR01000hz, icm.ACCEL_DLPF::ODR40);
    HAL_Delay(10);

    icm.GyroConfig(icm.GYRO_MODE::LowNoize, icm.GYRO_SCALE::Dps0250, icm.GYRO_ODR::ODR01000hz, icm.GYRO_DLPF::ODR40);
    HAL_Delay(10);

    return 0;
}

// キャリブレーション
uint8_t ICM42688P_SPI_Util::calibration(uint16_t calibration_count) {

    uint8_t calibration_error = icm.Calibration(calibration_count);

    if (calibration_error == 1) {

        return 1;
    }
    else if (calibration_error == 2) {

        return 2;
    }

    return 0;
}   

// データの取得
uint8_t ICM42688P_SPI_Util::getData(std::array<float, 3>& accel_data, std::array<float, 3>& gyro_data) {

    float accel[3] = {};
    float gyro[3]  = {};

    uint8_t result = icm.GetData(accel, gyro);

    for (uint8_t i = 0; i < 3; i++) {
        accel_data[i] = accel[i];
        gyro_data[i] = gyro[i];
    }

    return result;
}

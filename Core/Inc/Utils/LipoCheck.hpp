#ifndef LIPO_CHECK_HPP
#define LIPO_CHECK_HPP

#include "adc.h"
#include "gpio.h"

class LipoVoltageCheck {

    public:

        LipoVoltageCheck() = default;
        ~LipoVoltageCheck() = default;

        // 初期化とADCの開始
        void Setup(bool voltage_check, float min_voltage, ADC_HandleTypeDef* hadc) {

            this->voltage_check = voltage_check;
            this->min_voltage = min_voltage;
            this->hadc = hadc;

            if(hadc != nullptr) {

                HAL_ADC_Start(hadc);
            }
            else{

                this->voltage_check = false; // ADCがnullptrの場合、リポチェックを無効にする
            }
        }

        void SetupLED(GPIO_TypeDef* port, uint16_t pin) {

            use_led = true;
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); // LEDを消灯
        }

        void SetupBuzzer(GPIO_TypeDef* port, uint16_t pin) {

            use_buzzer = true;
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); // ブザーをオフ
        }

        // 電圧チェック
        bool CheckVoltage() {

            if (!voltage_check) {

                return true; // リポチェックが無効な場合、常にtrueを返す
            }

            // ADCの値を読む
            adc_value = HAL_ADC_GetValue(hadc);

            // 現在の電圧を計算
            voltage = (float)(adc_value / 4096 * 3.30f * 4.0f);

            HAL_ADC_Stop(hadc);

            HAL_ADC_Start(hadc);

            // 電圧が閾値以上の場合はtrueを返す
            if(voltage >= min_voltage) {

                return true;
            }
            else{

                if(use_led) {

                    HAL_GPIO_WritePin(led_port, led_pin, GPIO_PIN_SET); // LEDを点灯
                }

                if(use_buzzer) {

                    HAL_GPIO_WritePin(buzzer_port, buzzer_pin, GPIO_PIN_SET); // ブザーをオン
                }
            }

            return false;
        }

    private:

        // LEDの設定
        bool use_led = false;
        GPIO_TypeDef* led_port = nullptr;
        uint16_t led_pin = 0;

        // ブザーの設定
        bool use_buzzer = false;
        GPIO_TypeDef* buzzer_port = nullptr;
        uint16_t buzzer_pin = 0;

        // 電圧チェックの設定
        bool voltage_check = true;
        float min_voltage = 7.7f;

        // ADCと電圧の設定
        ADC_HandleTypeDef* hadc = nullptr;
        uint16_t adc_value = 0;
        float voltage = 0.0f;
};

#endif // LIPO_CHECK_HPP 
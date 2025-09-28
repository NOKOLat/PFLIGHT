#pragma once

#include <memory>
#include <iostream>

#include "state/interface/FlightStateInterface.h"
#include "FlightData/ControlData.hpp"
#include "UserSetting/SbusSetting.hpp"
#include "FlightData/SensorData.hpp"
#include "UserSetting/PIDSetting.hpp"
#include "UserSetting/ImuSetting.hpp"
#include "PID/PID.h"
#include "Utils/ICM42688P_SPI_Util.hpp"
#include "Utils/LED.hpp"
#include "MadgwickAHRS/src/MadgwickAHRS.h"
#include "Utils/MotorUtility.hpp"


// ループ管理構造体
struct FlightLoopManager{

    public:

        void setWaitFlag() {

            is_wait = true;
        }

        void clearWaitFlag() {

            is_wait = false;
        }

        bool isWait() const {

            return is_wait;
        }

    private:

        bool is_wait = false;
};

// フライト管理クラス
class FlightManager {

    public:

        FlightManager();
        ~FlightManager() = default;

        // 状態遷移
        void changeState(std::unique_ptr<FlightStateInterface> newState);
    
        // メインループ
        void update();

        // SBUSデータの更新（割り込み受信時に動作）
        void sbusUpdate(SbusChannelData sbus_data);

        bool checkSbusConnect();

        SensorData sensor_data;
        SbusChannelData sbus_data;
        ControlData control_data;

        PID angle_pitch;
        PID angle_roll;
        PID rate_pitch;
        PID rate_roll;
        PID rate_yaw;
        ICM42688P_SPI_Util* imuUtil;
        Madgwick madgwick;
		PWM_Quad pwm; //4Motor
        //PWM_Coaxial_Octa pwm; //8Motor
		LED red_led;
		LED yellow_led;
		LED green_led;

    private:

		std::unique_ptr<FlightStateInterface> current_state;
		uint16_t sbus_lost_count = 0;

};

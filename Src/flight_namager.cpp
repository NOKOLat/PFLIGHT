/*
 * flight_manager.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: Sezakiaoi
 */

#include "flight_manager.hpp"
#include "gpio.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"

#include "imu.hpp"
#include "PWM.hpp"
#include "PID_USER.hpp"
#include "MadgwickAHRS_USER.hpp"

bool sbus_updating = false;

// パブリックメソッド
void FlightManager::UpDate(){

	//戻り値を受け取る用の構造体のインスタンス
	StateResult result;

	//failsafeのチェック
	if(current_state != state::init){

		sbus_data.failsafe_count ++;

		if(sbus_data.failsafe_bit || sbus_data.failsafe_count > 250){

			//failsafe状態への移行
			current_state = state::failsafe;
		}
	}

    // 現在の状態に応じた処理を実行
    switch(current_state){
        case state::init:
            result = Init();
            break;

        case state::wait_arm:
            result = WaitArm();
            break;

        case state::arm:
            result = Arm();
            break;

        case state::wait_fly:
            result = WaitFly();
            break;

        case state::fly:
            result = Fly();
            break;

        case state::disarm:
            result = DisArm();
            break;

        case state::automation:
            result = Automation();
            break;

        case state::failsafe:
            result = FailSafe();

            while(1){

            	HAL_Delay(1000);
            	//無限ループ
            }
            break;

        default:
            // 未定義状態の場合はフェイルセーフへ遷移
            result.state_changed = true;
            result.next_state = state::failsafe;
            FailSafe();
            break;
    }

    //状態遷移をする場合の処理
    if(result.state_changed){

        current_state = result.next_state;
        printf("-----state_changed-----\n");
    }
}

void FlightManager::SbusUpDate(uint16_t sbus[10], bool failsafe_bit){

	sbus_updating = true;

	//FailSafeの処理
	sbus_data.failsafe_bit = failsafe_bit;

	//SBUSの受信チェック
	if(sbus[(uint8_t)Channel::arm] != 0){

		sbus_data.is_receive = true;
		sbus_data.failsafe_count = 0;
	}
	else{

		sbus_data.is_receive = false;
	}

	//pitch(angle ±30 dgree)
	sbus_data.target_pitch_angle = (float)((sbus[(uint8_t)Channel::pitch]    - sbus_data.center[(uint8_t)Channel::pitch]) / (float)(sbus_data.max[(uint8_t)Channel::pitch]    - sbus_data.center[(uint8_t)Channel::pitch])) * 30.0;

	//roll(angle ±30 dgree)
	sbus_data.target_roll_angle  = (float)((sbus[(uint8_t)Channel::roll]	 - sbus_data.center[(uint8_t)Channel::roll])  / (float)(sbus_data.max[(uint8_t)Channel::roll]     - sbus_data.center[(uint8_t)Channel::roll]))  * 30.0;

	//yaw(rate ±60 dps)
	sbus_data.target_yaw_rate    = (float)((sbus[(uint8_t)Channel::yaw]      - sbus_data.center[(uint8_t)Channel::yaw])   / (float)(sbus_data.max[(uint8_t)Channel::yaw] 	  - sbus_data.center[(uint8_t)Channel::yaw]))   * 60.0;

	//throttle(0~75%)
	sbus_data.throttle 			 = (float)((sbus[(uint8_t)Channel::throttle] - sbus_data.min[(uint8_t)Channel::throttle]) / (float)(sbus_data.max[(uint8_t)Channel::throttle] - sbus_data.min[(uint8_t)Channel::throttle])) * 750.0;

	//目標値を適切な配列に格納(PID計算で使用）
	sbus_data.target_angle[0] = sbus_data.target_pitch_angle;
	sbus_data.target_angle[1] = sbus_data.target_roll_angle;
	sbus_data.target_rate[2]  = sbus_data.target_yaw_rate;

	//armの判定
	if(sbus[(uint8_t)Channel::arm] > 1500){

		sbus_data.arm = true;
	}
	else{

		sbus_data.arm = false;
	}

	//飛行開始判定
	if(sbus[(uint8_t)Channel::fly] > 1500 && sbus[(uint8_t)Channel::throttle] < 400){

		sbus_data.fly = true;
	}
	else{

		sbus_data.fly = false;
	}

	//投下判定(2段階投下を採用）
	if(sbus[(uint8_t)Channel::drop] > 1500){

		sbus_data.drop = 2;
	}
	else if(sbus[(uint8_t)Channel::drop] > 1000){

		sbus_data.drop = 1;
	}
	else{

		sbus_data.drop = 0;
	}

	//自動投下判定
	if(sbus[(uint8_t)Channel::autodrop] > 1500){

		sbus_data.autodrop = true;
	}
	else{

		sbus_data.autodrop = false;
	}

	//自動操縦
	if(sbus[(uint8_t)Channel::autofly]){

		sbus_data.autofly = true;
	}
	else{

		sbus_data.autofly = false;
	}

	//printf("sbus %d  value %d \n ", sbus_data.is_receive, sbus[0]);
	//printf("sbus_data[throttle]: %d \n", sbus_data.throttle);

	sbus_updating =false;
}

state FlightManager::GetCurrentState(){

    return current_state;
}

// プライベートメソッド（状態ごとの処理）
StateResult FlightManager::Init(){

    StateResult result;

    //SBUSの受信チェック
    if(sbus_data.is_receive == false){

    	printf("sbus_error: %d\n", sbus_data.arm);

        result.error = error_state::SBUS_NOT_DETECTED;
        result.state_changed = false;

        return result;
    } 
        
    // IMUの受信チェック
    if(ImuInit() == 1){

        result.error = error_state::IMU_NOT_DETECTED;
        result.state_changed = false;

        printf("imu_error! \n");

        return result;
    }

    // 状態遷移用の処理
    result.error = error_state::NO_ERROR;
    result.state_changed = true;
    result.next_state = state::wait_arm;


    //Init終了LEDをつける
    printf("END: INIT() \n");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    return result;
}

StateResult FlightManager::WaitArm(){

    StateResult result;
    
    if(sbus_updating == true){

    	return result;
    }

    if(sbus_data.arm == false){

        result.error = error_state::ARM_NOT_DETECTED;
        result.state_changed = false;

        //printf("not_arm: %d \n", sbus_data.arm);

        return result;
    }

    // 状態遷移用の処理
    result.error = error_state::NO_ERROR;
    result.state_changed = true;
    result.next_state = state::arm;

    //Init終了LEDをつける
    printf("END: WaitArm() \n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

    return result;
}

StateResult FlightManager::Arm(){

    StateResult result;

    //ESCの初期化
    PwmInit();
    
    // 状態遷移用の処理
    result.error = error_state::NO_ERROR;
    result.state_changed = true;
    result.next_state = state::wait_fly;
    
    printf("END: ARM() \n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);


    return result;
}

StateResult FlightManager::WaitFly(){

    StateResult result;
    
    //Armが解除されていた場合→DisArmへ
    if(sbus_data.arm == false){

        result.error = error_state::ARM_NOT_DETECTED;
        result.state_changed = true;
        result.next_state = state::disarm;

        return result;
    }

    //飛行開始判定
    if(sbus_data.fly == false){

        result.error = error_state::FLY_NOT_DETECTED;
        result.state_changed = false;

        return result;
    }

    //PIDの初期化
    PidSetup();

　　//Madgwickの初期化
　　Madgwick_Start(400.0);
	
    // 状態遷移用の処理
    result.error = error_state::NO_ERROR;
    result.state_changed = true;
    result.next_state = state::fly;

    printf("END: WaitFly() \n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

    return result;
}

StateResult FlightManager::Fly(){

    StateResult result;
    static uint8_t fly_loop_count = 0;
    fly_loop_count ++;
    
    //Armのチェック
    if(sbus_data.arm == false){

        result.error = error_state::ARM_NOT_DETECTED;
        result.state_changed = true;
        result.next_state = state::disarm;

        fly_loop_count = 0;

        printf("END: Fly() \n");

        return result;
    }

    //センサーデータの取得
    ImuGetData(sensor_data.accel, sensor_data.gyro);

    //Madgickフィルターによる姿勢推定
    Madgwick_UpDate(sensor_data.accel, sensor_data.gyro);
    Madgwick_GetAngle(sensor_data.angle);

    //100hz 角度制御(pitch, roll)
    if(fly_loop_count % 4 == 0){

    	//目標角と現在角から目標角速度を計算
    	AnglePIDCalc(sensor_data.angle, sbus_data.target_angle);
    	AnglePIDGetData(sbus_data.target_rate);
    }

    //400hz 角速度制御
	//目標角速度と現在角速度(センサーデータ）から制御量を計算
	RatePIDCalc(sensor_data.gyro, sbus_data.target_rate);
	RatePIDGetData(control_data.pid_control);

	//PID結果を各モーターに分配
	CalcMotorPwm(sbus_data.throttle, control_data.pid_control, control_data.motor_pwm);

	//Servoのpwmを生成
	uint16_t adc_value = 0;

	//adcの値を読む
	CalcServoPwm(sbus_data, adc_value, control_data.servo_pwm);

	//PWMを生成
	PwmGenerate(control_data.motor_pwm, control_data.servo_pwm);

	printf("Motor: %d, %d, %d, %d \n", control_data.motor_pwm[0],control_data.motor_pwm[1],control_data.motor_pwm[2],control_data.motor_pwm[3]);

    // 状態遷移用の処理
    result.error = error_state::NO_ERROR;
    result.state_changed = false;

    if(sbus_data.autofly == true){


    }

    return result;
}

StateResult FlightManager::DisArm(){

    StateResult result;

    PwmStop();
    PidReset();
    
    static uint16_t delay = 0;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    //delay 1000ms (2.5s *400)
    while(delay < 400){

    	delay ++;

        result.error = error_state::DELAY_NOW;
        result.state_changed = false;

    	return result;
    }

    result.error = error_state::NO_ERROR;
    result.state_changed = true;
    result.next_state = state::wait_arm;

    printf("END: DisArm() \n");

    return result;
}

StateResult FlightManager::Automation(){
    StateResult result;
    
    // TODO: 自動飛行処理を実装
    
    return result;
}

StateResult FlightManager::FailSafe(){
    StateResult result;
    
    // TODO: フェイルセーフ処理を実装
    PwmStop();
    printf("FailSafe() \n");
    while(1){

    	HAL_Delay(500);

    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);


    }
    
    return result;
}

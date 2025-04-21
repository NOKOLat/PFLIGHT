#include "wrapper.hpp"

#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include "flight_data.hpp"
#include "flight_manager.hpp"
#include "interrupt.hpp"
#include "IMU.hpp"
#include "PWM.hpp"
#include "PID_USER.hpp"
#include "pose_estimation.h"
#include "SBUS.h"

SBUS sbus;

void init(){

	//----------FC起動----------//
	printf("-----Start FC-----\n");

	//PWMの停止
	PwmStop();

	//IMUの起動
	printf("IMU Status: %d/n", ImuInit());
}

void loop(){

	//----------Arm待機----------//
	printf("-----Wait Arm-----\n");

	//SBUSの受信開始(UART_DMA)
	SbusInit();

	//PWMの初期化
	PwmInit(motor.start, servo.close);

	//Arm待機
	while(IsArm()){

		//ArmLEDの点滅
		HAL_Delay(250);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);

		//debug
		printf("SbusError: %d\n", data.sbus[channel.arm]);
	}

	//ArmLEDの点灯
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

	//----------飛行待機----------//
	printf("-----Wait fly-----\n");

	//モーターを回転させる指示の待機
	while(IsStart());

	//最低回転数でモーターを回す
	PwmIdel(motor.min, servo.close);

	//PIDの初期化
	PidSetup();

	//ArmLoopとIMU割り込みの開始
	ArmIQRInit();

	//初回のデータ取得待機
	HAL_Delay(50);

	//----------ArmLoop----------//
	printf("-----ArmLoop-----\n");

	//Arm判定
	while(IsArm() == 0){

		//ArmLoop待機
		while(IsWaitLoop());

		//ArmLoopフラグをセット
		ArmLoopSet();

		//最新8つのセンターデータを取得
		float** accel_tmp = {};
		float** gyro_tmp = {};
		float** mag_tmp = {};

		data.accel.GetLatestValues2D(accel_tmp, 8);
		data.gyro.GetLatestValues2D(gyro_tmp, 8);

		//データを入力し計算
		for(uint8_t i=0; i<8; i++){

			PoseEstimation(accel_tmp[i], gyro_tmp[i], mag_tmp[i], data.angle, data.speed, data.position);
		}

		//目標角と現在角を入力
		PidCalc(data.angle, data.target_angle);

		//入力値から制御量を計算
		PidGetData(data.control);

		//各軸の制御量をモータに分配
		PidtoPwm();

		//モーターを回す
		PwmGenerate(data.motor, data.servo);
	}

	//----------DisArm----------//
	printf("-----DisArm-----\n");

	//PWMを止める
	PwmStop();

	//PIDのリセット
	PidReset();

	//ArmloopとImu割り込みを止める
	ArmIQRDeinit();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//FailSafe
	if(htim == &htim5){

		if(sbus.CheckFailsafe()){

			printf("-----FailSafe-----\n");

			//PWMを止める
			PwmStop();

			//割り込みを止める
			SbusDeInit();
			ArmIQRDeinit();
			FailsafeDeInit();

			//無限ループでスタックさせる
			while(1);
		}
	}

	//MainLoop
	if(htim == &htim6){

		//ArmLoopフラグをクリア
		ArmLoopClear();
	}

	//IMU
	if(htim == &htim7){

		//データ取得用の仮変数
		float accel_tmp[3] = {};
		float gyro_tmp[3]  = {};

		//データを取得
		ImuGetData(accel_tmp, gyro_tmp);

		//データをRingBufferに入力
		data.accel.SetValue(accel_tmp);
		data.gyro.SetValue(gyro_tmp);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	//SBUS
	if(huart == &huart5){

		//受信チェックとエンコード
		if(sbus.IsSbus() == 0){

			sbus.SetRawData(data.raw_sbus);
			sbus.Encode();
			sbus.GetData(data.sbus);

			//目標角を計算
			SbustoAngle();
		}

		//受信の再開
		SbusInit();
	}
}

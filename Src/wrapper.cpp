#include "wrapper.hpp"

#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include "MadgwickAHRS_USER.hpp"

#include "flight_data.hpp"
#include "flight_manager.hpp"
#include "interrupt.hpp"
#include "IMU.hpp"
#include "PWM.hpp"
#include "PID_USER.hpp"
#include "SBUS.h"
#include "Debug.hpp"


void FailSafe();

//-----Instance-----//

SBUS sbus;

//-----FC Init-----//
void init(){

	printf("-----FC Init-----\n");

	//SBUSの受信開始(UART_DMA)
	uint16_t error = 0;

	//受信するまで再起動する
	while(data.sbus[channel.arm] == 0){

		//初期化LEDを点滅させる
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);

		SbusDeInit(&huart5);

		HAL_Delay(29);

		SbusInit(&huart5, data.raw_sbus);

		error ++;

		//失敗した場合はエラーメッセージを表示する
		if(error > 100){

			printf("SbusError: DisConnect\n");
		}
	}

	printf("IMU Status: %d\n", ImuInit());
	Madgwick_Start(500);

	//初期化終了LEDをつける
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}

//-----FC MainLoop-----//
void loop(){

	//PWMの初期化
	MotorInit();

	//----------Arm待機----------//
	printf("-----Wait Arm-----\n");

	//Arm待機
	while(IsArm()){

		//ArmLEDの点滅
		HAL_Delay(250);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

		//debug
		printf("SbusError: %d\n", data.sbus[channel.arm]);
	}

	//ArmLEDの点灯
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	//----------飛行待機----------//
	printf("-----Wait fly-----\n");

	//モーターを回転させる指示の待機
	//DisArmした場合は何も実行されずに、DisArm処理まで進む
	//以降すべての処理にArm判定があるため
	while(IsStart() == 1 && IsArm() == 0){

		//待機用のLEDを点滅させる
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
		HAL_Delay(250);
	}

	//Armしている場合のみ実行
	if(IsArm() == 0){

		//待機用のLEDを点滅させる
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

		//PIDの初期化
		PidSetup();

		//ArmLoopとIMU割り込みの開始
		ArmIQRInit();

		//初回のデータ取得待機
		HAL_Delay(100);
	}
	//----------ArmLoop----------//
	if(IsArm() == 0){

		printf("-----ArmLoop-----\n");
	}

	//Arm判定
	while(IsArm() == 0){

		//ArmLoop待機
		while(IsWaitLoop()){

			//フェイルセーフの判定
			if(sbus.CheckFailsafe()){

				FailSafe();
			}
		}

		//ArmLoopフラグをセット
		ArmLoopSet();

		//SBUSのデータを取得
		sbus.GetData(data.sbus);

		//SBUSのデータから目標角を計算
		//data.target_angleに目標角が入る
		SbustoAngle();

		//センサーデータを取得
		ImuGetData(data.accel, data.gyro);

		//センサーデータをMadgwickフィルターに入力して計算
		Madgwick_UpDate(data.accel, data.gyro, data.mag);

		//Madgwickフィルターから角度を取得
		Madgwick_GetAngle(data.angle);

		//現在角と目標角からPIDの値を計算
		PidCalc(data.angle, data.target_angle);

		//PIDの値を取得
		PidGetData(data.control);

		//PIDの値をPWMに変換
		PidtoPwm(data.throttle, data.control, data.motor);

		//PWMの値を使ってモーターを回す
		MotorGenerate(data.motor, data.servo);

		//Debug
		//SendData(data.motor);
	}

	//----------DisArm----------//

	printf("-----DisArm-----\n");

	//PWMを止める
	MotorStop();

	//ArmloopとImu割り込みを止める
	ArmIQRDeinit();

	//LEDを消す
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//MainLoop
	if(htim == &htim6){

		//ArmLoopフラグをクリア
		ArmLoopClear();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	//SBUS
	if(huart == &huart5){

		//受信チェックとエンコード
		sbus.SetRawData(data.raw_sbus);
		sbus.Encode();

		//受信の再開
		SbusInit(&huart5, data.raw_sbus);
	}
}

void FailSafe(){

	printf("-----FailSafe-----\n");

	//PWMを止める
	MotorStop();

	//割り込みを止める
	SbusDeInit(&huart5);
	ArmIQRDeinit();

	//無限ループでスタックさせる
	while(1){

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
		HAL_Delay(250);
	}
}

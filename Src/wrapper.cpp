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

//#define NO_SBUS_MODE //デバック用　危険なのでモーターが回らないようにすること
#define SHOW_STATUS  //デバック用　PWMの値を出力

void FailSafe();

//-----Instance-----//

SBUS sbus;

//-----FC Init-----//
void init(){

	printf("-----FC Init-----\n");

	//SBUSの受信開始(UART_DMA)
	uint16_t error = 0;

	//SBUSの受信開始
	SbusInit(&huart5, data.raw_sbus);

	HAL_Delay(100);

	//受信するまで再起動する
	while(data.sbus[channel.arm] == 0){

		//初期化LEDを点滅させる
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);

		SbusDeInit(&huart5);

		HAL_Delay(10);

		SbusInit(&huart5, data.raw_sbus);

		HAL_Delay(25);

		error ++;

		//失敗した場合はエラーメッセージを表示する
		if(error > 100){

			printf("SbusError: DisConnect\n");
		}

		#ifdef NO_SBUS_MODE

			data.sbus[channel.arm] = 497;
		#endif
	}

	printf("Sbus Ready \n");
	HAL_Delay(100);

	//IMUの初期化
	while(ImuInit() == 1){

		printf("IMU Error: Please Restart\n");

		HAL_Delay(100);
	}

	//Madgwickフィルターの初期化
	Madgwick_Start(400);

	//初期化終了LEDをつける
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}

//-----FC MainLoop-----//
void loop(){

	//PWMの初期化
	PwmInit();

	//----------Arm待機----------//
	printf("-----Wait Arm-----\n");

	//Arm待機
	while(IsArm()){

		//SBUSのデータを取得
		sbus.GetData(data.sbus);

		//ArmLEDの点滅
		HAL_Delay(250);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);


		#ifdef NO_SBUS_MODE

			data.sbus[channel.arm] = 2000;
			printf("DebugMode ArmChannel: %d\n", data.sbus[channel.arm]);
		#endif
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

		#ifdef NO_SBUS_MODE

			data.sbus[channel.start] = 2000;
			printf("DebugMode StartChannel: %d\n", data.sbus[channel.start]);
		#endif
	}

	//Armしている場合のみ実行
	if(IsArm() == 0){

		//待機用のLEDを点滅させる
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

		//PIDの初期化
		//PidSetup();

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

		//SBUSのデータを取得（割り込みで実装済み）
		//sbus.GetData(data.sbus);

		//SBUSのデータから目標角を計算
		//data.target_angleに目標角が入る
		SbustoAngle();

		//SBUSがないときは指定値を入力
		#ifdef NO_SBUS_MODE

			data.target_angle[0] = 0;
			data.target_angle[1] = 0;
			data.target_angle[2] = 0;
			data.throttle = 300;
			data.sbus[channel.arm] = 2000;
		#endif

		//センサーデータを取得
		ImuGetData(data.accel, data.gyro);

		//センサーデータをMadgwickフィルターに入力して計算
		Madgwick_UpDate(data.accel, data.gyro, data.mag);

		//Madgwickフィルターから角度と角速度を取得
		Madgwick_GetData(data.angle, data.rate);

		//角度制御ループ(100hz)
		if(armloop.loop_count % 4 == 0){

			//現在角と目標角から目標角速度を計算
			AnglePIDCalc(data.angle, data.target_angle);

			//角度PIDの出力（目標角速度）を取得
			AnglePIDGetData(data.target_rate);
		}

		//角速度制御ループ(400hz)
		//現在角速度と目標角速度からPIDの計算
		RatePIDCalc(data.rate, data.target_rate);

		//角速度PIDの出力（PWMの制御量）を取得
		RatePIDGetData(data.control);

		//PIDの値をPWMに変換
		PidtoPwm(data.throttle, data.control, data.motor);

		//PWMの値を使ってモーターを回す
		PwmGenerate(data.motor, data.servo);

		#ifdef SHOW_STATUS

			//SendData(data.motor);
			//SendData(data.angle);
			//SendData(data.angle_speed);
			//SendData(data.gyro);
			//SendData(data.control);
		#endif
	}

	//----------DisArm----------//

	printf("-----DisArm-----\n");

	//PWMを止める
	PwmStop();

	//ArmloopとImu割り込みを止める
	ArmIQRDeinit();

	HAL_Delay(1000);

	//LEDを消す
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//MainLoop
	if(htim == &htim6){

		//ArmLoopフラグをクリア
		//ついでにループカウントを管理
		ArmLoopClear();
	}
	else if(htim == &htim7){

		#ifdef NO_SBUS_MODE

		armloop.failsafe_count ++;

		if(armloop.failsafe_count > 100){

			FailSafe();
		}

		#endif
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	//SBUS
	if(huart == &huart5){

		//受信チェックとエンコード
		sbus.SetRawData(data.raw_sbus);
		sbus.Encode();

		sbus.GetData(data.sbus);

		//受信の再開
		SbusInit(&huart5, data.raw_sbus);

		armloop.failsafe_count = 0;
	}
}

void FailSafe(){

	printf("-----FailSafe-----\n");

	//PWMを止める
	PwmStop();

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

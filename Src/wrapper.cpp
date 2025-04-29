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

		SbusDeInit();

		HAL_Delay(29);

		SbusInit();

		error ++;

		//失敗した場合はエラーメッセージを表示する
		if(error > 100){

			printf("SbusError: DisConnect\n");
		}
	}

	printf("IMU Status: %d\n", ImuInit());

	//PWMの初期化
	PwmInit(motor.init, servo.close);


	//初期化終了LEDをつける
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}

//-----FC MainLoop-----//
void loop(){

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

			if(sbus.CheckFailsafe()){

				printf("-----FailSafe-----\n");

				//PWMを止める
				PwmStop();

				//割り込みを止める
				SbusDeInit();
				ArmIQRDeinit();
				FailsafeDeInit();



				//無限ループでスタックさせる
				while(1){

					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
					HAL_Delay(250);
				}
			}
		}

		//ArmLoopフラグをセット
		ArmLoopSet();

		//データを入力する
		for(uint8_t i=0; i<8; i++){

			//偶数ループ
			if(armloop.loop_count % 2 == 0){

				PoseEstimation(data.accel[i], data.gyro[i], data.mag[i], data.angle, data.speed, data.position);
			}
			else{

				PoseEstimation(data.accel[i+8], data.gyro[i+8], data.mag[i+8], data.angle, data.speed, data.position);
			}
		}

		//printf("%3.3lf %3.3lf %3.3lf\n", data.gyro[0][0], data.gyro[0][1], data.gyro[0][2]);

		//printf("%3.3lf %3.3lf %3.3lf\n", data.angle[0], data.angle[1], data.angle[2]);
		//printf("%+4.4lf %+4.4lf %+4.4lf %+4d \n", data.target_angle[0], data.target_angle[1], data.target_angle[2], data.throttle);

		PidCalc(data.angle, data.target_angle);
		PidGetData(data.control);

		PidtoPwm();

		printf("%+4d %+4d %+4d %+4d\n", data.motor[0], data.motor[1], data.motor[2], data.motor[3]);
		PwmGenerate(data.motor, data.servo);

		//printf("%+4.4lf %+4.4lf %+4.4lf %+4d \n", data.target_angle[0], data.target_angle[1], data.target_angle[2], data.throttle);
	}

	//----------DisArm----------//

	printf("-----DisArm-----\n");

	//PWMを止める
	PwmStop();

	//ArmloopとImu割り込みを止める
	ArmIQRDeinit();

	//LEDを消す
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

	HAL_Delay(1000);
	PwmInit(motor.init, servo.close);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//MainLoop
	if(htim == &htim6){

		//ArmLoopフラグをクリア
		ArmLoopClear();
	}

	//IMU
	if(htim == &htim7){

		//データが8個集まっていないときのみ処理
		if(armloop.data_index < 8){

			//偶数ループ
			if(armloop.loop_count % 2 == 0){

				ImuGetData(data.accel[armloop.data_index + 8], data.gyro[armloop.data_index + 8]);
				armloop.data_index ++;
			}
			else{
				ImuGetData(data.accel[armloop.data_index], data.gyro[armloop.data_index]);
				armloop.data_index ++;
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	//SBUS
	if(huart == &huart5){

		//受信チェックとエンコード

		sbus.SetRawData(data.raw_sbus);
		sbus.Encode();
		sbus.GetData(data.sbus);

		SbustoAngle();

		//受信の再開
		SbusInit();
	}
}

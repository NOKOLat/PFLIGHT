#include "wrapper.hpp"
#include "pflight.hpp"
#include <stdio.h>

void init(){

	printf("IMU: %d\n",init_imu());
}

void loop(){

	//----------Arm待機----------//

	//SBUSの受信開始(LED消灯）
	init_sbus();

	//PWMの初期化
	init_pwm();

	//arm待機(LED点滅)
	while(!is_arm()){

		HAL_Delay(500);
	}

	//----------飛行待機----------//

	//モーター待機(LED点灯）
	while(!is_rotate());

	//モーターを回転（最小回転数）
	idel_pwm();

	//pidのスタート
	pid_setup();

	//飛行割り込みの開始
	init_timer_interrupt();

	//----------ArmLoop----------/

	while(is_arm()){

		//指定時間まで待機
		while(is_wait_armloop());

		set_wait_flag();

		calc_pose();

		clear_value();

		pid_calc();

		pid_to_pwm();

		generate_pwm();

		printvalue();
	}

	//----------DisArm処理----------//

	//PWMの停止
	void stop_pwm();
	void deinit_pwm();

	//割り込みの停止
	deinit_timer_interrupt();
	deinit_sbus();

	HAL_Delay(1000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//FailSafe
	if(htim == &htim5){

//		if(check_failsafe()){
//
//			printf("-----failsafe-----\n");
//
//			HAL_Delay(65535);
//		}
	}

	//MainLoop
	if(htim == &htim6){

		init_armloop();
	}

	//IMU
	if(htim == &htim7){

		get_imu_data();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart == &huart2){

		encode_sbus();
		init_sbus();
	}
}


#include "wrapper.hpp"
#include "pflight.hpp"
#include <stdio.h>

void init(){

	//SBUSの受信開始(LED消灯）
	init_sbus();

	printf("IMU: %d\n",init_imu());
	//debug_mode();
}

void loop(){

	//----------Arm待機----------//
	printf("-----Wait Arm-----\n");

	//PWMの初期化
	init_pwm();

	//arm待機(LED点滅)
	while(!is_arm()){

		HAL_Delay(500);
	}

	//----------飛行待機----------//

	printf("-----Wait fly-----\n");

	//モーター待機(LED点灯）
	while(!is_rotate());

	//モーターを回転（最小回転数）
	idel_pwm();

	//飛行割り込みの開始
	init_timer_interrupt();

	//----------ArmLoop----------/

	printf("-----ArmLoop-----\n");

	while(is_arm()){

		//指定時間まで待機
		while(is_wait_armloop());

		if(check_failsafe()){

			printf("-----failsafe-----\n");

			stop_pwm();

			while(1);
		}

		//printf("---mainloop---\n");

		set_wait_flag();

		calc_pose();

		clear_value();

		pid_calc();

		pid_to_pwm();

		generate_pwm();

		printvalue();
	}

	//----------DisArm処理----------//
	printf("-----DisArm-----\n");

	//PWMの停止
	stop_pwm();

	//PIDの停止
	pid_reset();

	//割り込みの停止
	deinit_timer_interrupt();
	//deinit_sbus();

	HAL_Delay(1000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

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

	if(huart == &huart5){

		encode_sbus();
		sbus_to_angle();

		init_sbus();
	}

}


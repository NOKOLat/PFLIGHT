#include "wrapper.hpp"
#include "pflight.hpp"
#include <stdio.h>

void init(){

	printf("IMU: %d\n",init_imu());

	//debug_mode();
}

void loop(){

	//----------Arm待機----------//
	printf("-----Wait Arm-----\n");

	//SBUSの受信開始(LED消灯）
	init_sbus();

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

	//pidのスタート
	pid_setup();

	//飛行割り込みの開始
	init_timer_interrupt();

	//----------ArmLoop----------/

	printf("-----ArmLoop-----\n");

	while(is_arm()){

		//指定時間まで待機
		while(is_wait_armloop());

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
	void stop_pwm();
	void deinit_pwm();

	//割り込みの停止
	deinit_timer_interrupt();
	//deinit_sbus();

	HAL_Delay(1000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//FailSafe
	if(htim == &htim5){

		if(check_failsafe()){


			printf("-----failsafe-----\n");
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_Delay(65535);
		}
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

	if(huart == &huart1){

		encode_sbus();
		sbus_to_angle();
	}

	init_sbus();
}


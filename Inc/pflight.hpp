/*
 * pflight.hpp
 *
 *  Created on: Mar 31, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_PFLIGHT_HPP_
#define INC_PFLIGHT_HPP_

#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "spi.h"

//--Arm--//
uint8_t is_arm();
uint8_t is_rotate();
uint8_t is_auto();

//--メインループ管理--//
void init_armloop();
void set_wait_flag();
uint8_t is_wait_armloop();

//--タイマー割り込み管理--//
void init_timer_interrupt();
void deinit_timer_interrupt();

//--SBUS管理とフェイルセーフ--//
void init_sbus();
void deinit_sbus();
void encode_sbus();
void sbus_to_angle();
uint8_t check_failsafe();

//--IMU--//
uint8_t init_imu();
void get_imu_data();

//--姿勢推定--//
void calc_pose();
void clear_value();

//--PID--//

void pid_setup();
void pid_calc();
void pid_to_pwm();
//--PWM--//
void init_pwm();
void idel_pwm();
void stop_pwm();
void deinit_pwm();
void generate_pwm();

//--Debug--//
void debug_mode();
void printvalue();

#endif /* INC_PFLIGHT_HPP_ */

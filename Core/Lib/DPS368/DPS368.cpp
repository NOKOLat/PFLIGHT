/*
 * DPS368.cpp
 *
 *  Created on: Aug 17, 2025
 *      Author: Sezakiaoi
 */

#include "DPS368.hpp"
#include <stdio.h>


uint8_t DPS368::init(){

	//ProductIDの確認（通信チェック）
	uint8_t productId = 0x00;
	uint8_t errorCount = 0;

	//100回まで試行して失敗した場合はreturn 1
	//productIdは16(10進数）
	while(productId != 16){

		RegRead(DPS368_Register::Product_ID, &productId, 1);

		if(errorCount++ > 100){

			return 1;
		}
	}

	// 連続測定モードの設定
	uint8_t meas_cfg_command = 0b111; //連続測定モード
	uint8_t meas_cfg_value = 0x00;    //値を読み取る
	errorCount = 0;

	//100回まで試行して失敗した場合はreturn 2
	//上位5桁は読み取り専用であるため、下位3桁のみで判定
	while(meas_cfg_command != (meas_cfg_value & 0b111)){

		RegWrite(DPS368_Register::MEAS_CFG, &meas_cfg_command, 1);

		RegRead(DPS368_Register::MEAS_CFG, &meas_cfg_value, 1);

		if(errorCount++ > 100){

			return 2;
		}
	}

	// 係数を一度読み込んで保存しておく（getData で毎回読み込ませない）
	uint8_t ret = ReadCoefficients();
	if(ret != 0){
		return 3; // 係数読み込み失敗
	}

	// Configure pressure and temperature measurement to higher rates/sampling
	// so that new measurements are available at higher frequency.
	// Use max measurement rate and moderate oversampling by default.
	uint8_t cfg_ret = pressConfig(MEAS_RATE::_128pr_sec, MEAS_SAMPLING::_004_times);
	if(cfg_ret != 0){
		// non-fatal: continue but notify
		printf("DPS368: pressConfig failed\n");
	}
	cfg_ret = tempConfig(MEAS_RATE::_128pr_sec, MEAS_SAMPLING::_004_times);
	if(cfg_ret != 0){
		printf("DPS368: tempConfig failed\n");
	}

	return 0;
}

uint8_t DPS368::pressConfig(MEAS_RATE rate, MEAS_SAMPLING sampling){

	// rate[6:4] + sampling[3:0]
	uint8_t command = (uint8_t)rate << 4 | (uint8_t)sampling;

	uint8_t ret = RegWrite(DPS368_Register::PRS_CFG, &command, 1);

	if(ret != 0){

		return 1;
	}

	//校正計算用の値の保存(Compensation Scale Factors)
	switch ((uint8_t)sampling) {
	case 0:   press_compensation_scale_factors = 524288;  break;
	case 1:   press_compensation_scale_factors = 1572864; break;
	case 2:   press_compensation_scale_factors = 3670016; break;
	case 3:   press_compensation_scale_factors = 7864320; break;
	case 4:   press_compensation_scale_factors = 253952;  break;
	case 5:   press_compensation_scale_factors = 516096;  break;
	case 6:   press_compensation_scale_factors = 1040384; break;
	case 7:   press_compensation_scale_factors = 2088960; break;
	default:  press_compensation_scale_factors = 0;       break;
}

	return 0;
}

uint8_t DPS368::tempConfig(MEAS_RATE rate, MEAS_SAMPLING sampling){

	// rate[6:4] + sampling[3:0]
	uint8_t command = (uint8_t)rate << 4 | (uint8_t)sampling;

	uint8_t ret = RegWrite(DPS368_Register::TMP_CFG, &command, 1);

	if(ret != 0){

		return 1;
	}

	//校正計算用の値の保存(Compensation Scale Factors)
	switch ((uint8_t)sampling) {
	case 0:   temp_compensation_scale_factors = 524288;  break;
	case 1:   temp_compensation_scale_factors = 1572864; break;
	case 2:   temp_compensation_scale_factors = 3670016; break;
	case 3:   temp_compensation_scale_factors = 7864320; break;
	case 4:   temp_compensation_scale_factors = 253952;  break;
	case 5:   temp_compensation_scale_factors = 516096;  break;
	case 6:   temp_compensation_scale_factors = 1040384; break;
	case 7:   temp_compensation_scale_factors = 2088960; break;
	default:  temp_compensation_scale_factors = 0;       break;
}

	return 0;
}

// Read calibration coefficients from sensor and store in private members
uint8_t DPS368::ReadCoefficients(){

	uint8_t buffer[18] = {};

	uint8_t ret = RegRead(DPS368_Register::COEF, buffer, 18);

	if(ret != 0){
		return 1;
	}

	// compose coefficients from buffer content and store to members (snake_case)
	m_c0_half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
	GetTwosComplement(&m_c0_half, 12);
	m_c0_half = m_c0_half / 2U;

	m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
	GetTwosComplement(&m_c1, 12);

	m_c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
	GetTwosComplement(&m_c00, 20);

	m_c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
	GetTwosComplement(&m_c10, 20);

	m_c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
	GetTwosComplement(&m_c01, 16);

	m_c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
	GetTwosComplement(&m_c11, 16);

	m_c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
	GetTwosComplement(&m_c20, 16);

	m_c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
	GetTwosComplement(&m_c21, 16);

	m_c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
	GetTwosComplement(&m_c30, 16);

	// mark that coefficients are loaded
	coeffs_loaded = true;

	return 0;
}

uint8_t DPS368::updateCoefficients(){
	// simply call ReadCoefficients to refresh saved coefficients
	uint8_t ret = ReadCoefficients();
	return ret;
}

// Data retrieval: uses coefficients read during init()
uint8_t DPS368::getData(float * pressData, float * tempData){

	//生のセンサーデータの取得
	uint8_t rawData[6] = {};
	uint8_t ret = RegRead(DPS368_Register::PSR_B2, rawData, 6);

	if(ret != 0){
		return 2;
	}

	int32_t rawPress = ((uint32_t)rawData[0] << 16 | (uint32_t)rawData[1] << 8 | (uint32_t)rawData[2]);
	GetTwosComplement(&rawPress, 24);

	int32_t rawTemp  = ((uint32_t)rawData[3] << 16 | (uint32_t)rawData[4] << 8 | (uint32_t)rawData[5]);
	GetTwosComplement(&rawTemp, 24);

	float rawPressSC = (float)((float)rawPress / press_compensation_scale_factors);
	float rawTempSC = (float)((float)rawTemp / temp_compensation_scale_factors);

	// 圧力[Pa]の計算
	pressData[0] = (float)(m_c00 + rawPressSC * (m_c10 + rawPressSC * (m_c20 + rawPressSC * m_c30)) + rawTempSC * (m_c01 + rawPressSC * (m_c11 + rawPressSC * m_c21)));

	// 温度[℃]の計算
	tempData[0] = m_c0_half + m_c1 * rawTempSC + 30.0;

	return 0;
}

void DPS368::GetTwosComplement(int32_t *raw, uint8_t length){

	if (*raw & ((uint32_t)1 << (length - 1))){
		*raw -= (uint32_t)1 << length;
	}
}

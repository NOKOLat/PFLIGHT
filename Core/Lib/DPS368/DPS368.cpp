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
	    case 0:   pressCompensationScaleFactors = 524288;  break;
	    case 1:   pressCompensationScaleFactors = 1572864; break;
	    case 2:   pressCompensationScaleFactors = 3670016; break;
	    case 3:   pressCompensationScaleFactors = 7864320; break;
	    case 4:   pressCompensationScaleFactors = 253952;  break;
	    case 5:   pressCompensationScaleFactors = 516096;  break;
	    case 6:   pressCompensationScaleFactors = 1040384; break;
	    case 7:   pressCompensationScaleFactors = 2088960; break;
	    default:  pressCompensationScaleFactors = 0;       break;
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
	    case 0:   tempCompensationScaleFactors = 524288;  break;
	    case 1:   tempCompensationScaleFactors = 1572864; break;
	    case 2:   tempCompensationScaleFactors = 3670016; break;
	    case 3:   tempCompensationScaleFactors = 7864320; break;
	    case 4:   tempCompensationScaleFactors = 253952;  break;
	    case 5:   tempCompensationScaleFactors = 516096;  break;
	    case 6:   tempCompensationScaleFactors = 1040384; break;
	    case 7:   tempCompensationScaleFactors = 2088960; break;
	    default:  tempCompensationScaleFactors = 0;       break;
	}

	return 0;
}

uint8_t DPS368::getData(float * pressData, float * tempData){

	//センサー状態の取得
	uint8_t sensorState = 0x00;
	RegRead(DPS368_Register::MEAS_CFG, &sensorState, 1);


	// 校正の変数
	int32_t m_c0Half;
	int32_t m_c1;
	int32_t m_c00;
	int32_t m_c10;
	int32_t m_c01;
	int32_t m_c11;
	int32_t m_c20;
	int32_t m_c21;
	int32_t m_c30;

	//校正計算ができる場合

	uint8_t buffer[18] = {};

	uint8_t ret = RegRead(DPS368_Register::COEF, buffer, 18);

	if(ret != 0){

		return 1;
	}

    // compose coefficients from buffer content
    m_c0Half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
    getTwosComplement(&m_c0Half, 12);
    // c0 is only used as c0*0.5, so c0_half is calculated immediately
    m_c0Half = m_c0Half / 2U;

    // now do the same thing for all other coefficients
    m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
    getTwosComplement(&m_c1, 12);
    m_c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
    getTwosComplement(&m_c00, 20);
    m_c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
    getTwosComplement(&m_c10, 20);

    m_c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
    getTwosComplement(&m_c01, 16);

    m_c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
    getTwosComplement(&m_c11, 16);
    m_c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
    getTwosComplement(&m_c20, 16);
    m_c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
    getTwosComplement(&m_c21, 16);
    m_c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
    getTwosComplement(&m_c30, 16);

    //センターデータが取得できる場合

	//生のセンサーデータの取得
	uint8_t rawData[6] = {};
	ret = 0;
	ret = RegRead(DPS368_Register::PSR_B2, rawData, 6);

	if(ret != 0){

		return 2;
	}

	//データの結合
	int32_t rawPress = ((uint32_t)rawData[0] << 16 | (uint32_t)rawData[1] << 8 | (uint32_t)rawData[2]);
	getTwosComplement(&rawPress, 24);

	int32_t rawTemp  = ((uint32_t)rawData[3] << 16 | (uint32_t)rawData[4] << 8 | (uint32_t)rawData[5]);
	getTwosComplement(&rawTemp, 24);

	//スケーリング
	float rawPressSC = (float)((float)rawPress / pressCompensationScaleFactors);
	float rawTempSC = (float)((float)rawTemp / tempCompensationScaleFactors);

	//圧力[Pa]の計算
	pressData[0] = (float)(m_c00 + rawPressSC * (m_c10 + rawPressSC * (m_c20 + rawPressSC * m_c30)) + rawTempSC * (m_c01 + rawPressSC * (m_c11 + rawPressSC * m_c21)));

	//温度[℃]の計算
	tempData[0] = m_c0Half + m_c1 * rawTempSC + 30.0;



	return 0;
}

void DPS368::getTwosComplement(int32_t *raw, uint8_t length){

    if (*raw & ((uint32_t)1 << (length - 1))){

        *raw -= (uint32_t)1 << length;
    }
}

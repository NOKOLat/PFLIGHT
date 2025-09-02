/*
 * DPS368_HAL_I2C.cpp
 *
 *  Created on: Aug 17, 2025
 *      Author: Sezakiaoi
 */



#include "DPS368_HAL_I2C.hpp"
#include <stdio.h>

DPS368_HAL_I2C::DPS368_HAL_I2C(I2C_HandleTypeDef* i2cPin){

    this->i2cPin = i2cPin;
}

uint8_t DPS368_HAL_I2C::RegWrite(DPS368_Register regAddr, uint8_t* txBuffer, uint8_t dataLen){

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(i2cPin, i2cAddr, uint8_t(regAddr), 1, txBuffer, dataLen, 100);

    if(ret == HAL_OK){

    	return 0;
    }
    else{

    	printf("i2c_error\n");
    	return 1;
    }
}

uint8_t DPS368_HAL_I2C::RegRead(DPS368_Register regAddr, uint8_t* receiveBuffer, uint8_t dataLen){

	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(i2cPin, i2cAddr, uint8_t(regAddr), 1, receiveBuffer, dataLen, 100);

    if(ret == HAL_OK){

    	return 0;
    }
    else{

    	printf("i2c_error\n");
    	return 1;
    }

    return 0;
}

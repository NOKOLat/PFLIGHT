/*
 * FIR.h
 *
 *  Created on: Feb 21, 2025
 *      Author: takut
 */

#ifndef INC_CALCULATION_FIR_H_
#define INC_CALCULATION_FIR_H_

#define NUM_TAPS 8
#define BLOCK_SIZE 32

typedef struct {
	float in[BLOCK_SIZE],out[BLOCK_SIZE];
	float pState[NUM_TAPS+BLOCK_SIZE-1];
}FIR; FIR fir[3];

const float pCoeffs[NUM_TAPS]= {
	0.02069113757759074,
	0.06555078083899221,
	0.16641303731126522,
	0.24734504427215181,
	0.24734504427215181,
	0.16641303731126522,
	0.06555078083899221,
	0.02069113757759074
};

void FIR_set(float data[3]){//観測値を保存する
	for(uint8_t i=0;i<BLOCK_SIZE-1;i++){
		fir[0].in[i] = fir[0].in[i+1];
		fir[1].in[i] = fir[1].in[i+1];
		fir[2].in[i] = fir[2].in[i+1];
	}
	fir[0].in[BLOCK_SIZE-1] = data[0];
	fir[1].in[BLOCK_SIZE-1] = data[1];
	fir[2].in[BLOCK_SIZE-1] = data[2];
}

void FIR_calc(float data[3]){//保存されたデータにFIRをかけ、引数に出力する
	for (uint8_t i=0;i<3;i++){
		arm_fir_instance_f32 S2 = {NUM_TAPS, fir[i].pState,pCoeffs};
		arm_fir_f32 (&S2,fir[i].in,fir[i].out,BLOCK_SIZE);
	}
	data[0] = fir[0].out[BLOCK_SIZE-1];
	data[1] = fir[1].out[BLOCK_SIZE-1];
	data[2] = fir[2].out[BLOCK_SIZE-1];

}
#endif /* INC_CALCULATION_FIR_H_ */

/*
 * FIR.c
 *
 *  Created on: Aug 11, 2025
 *      Author: takut
 * 
 * ARM DSP FIRライブラリの正しい使用方法:
 * 
 * 1. arm_fir_init_f32(): FIRインスタンスを初期化
 *    - numTaps: フィルタ係数の数
 *    - pCoeffs: フィルタ係数配列
 *    - pState: ステートバッファ (サイズ: numTaps + blockSize - 1)
 *    - blockSize: 1回の処理で扱うサンプル数（参考値）
 * 
 * 2. arm_fir_f32(): FIR処理実行
 *    - S: FIRインスタンス
 *    - pSrc: 入力データ配列
 *    - pDst: 出力データ配列  
 *    - blockSize: 実際に処理するサンプル数（初期化時と異なっても可）
 * 
 * 注意: ARM DSPライブラリは単一サンプル処理には最適化されていない
 *       単一サンプルが必要な場合は手動実装を使用
 */

#include "FIR.h"
#include "arm_math.h"
#include <math.h>

// FIRフィルタのグローバルインスタンス
FIR fir[3];

// FIRインスタンス（ARM DSP用）
static arm_fir_instance_f32 fir_instance[3];

// 初期化済みフラグ
static uint8_t fir_initialized = 0;

const float pCoeffs[NUM_TAPS]= {
	0.02069113757759074f,
	0.06555078083899221f,
	0.16641303731126522f,
	0.24734504427215181f,
	0.24734504427215181f,
	0.16641303731126522f,
	0.06555078083899221f,
	0.02069113757759074f
};

void FIR_init(void) {
	// FIRフィルタの初期化（3軸分）
	for(uint8_t i = 0; i < 3; i++) {
		// 入力バッファとステートバッファをゼロクリア
		for(uint8_t j = 0; j < BLOCK_SIZE; j++) {
			fir[i].in[j] = 0.0f;
			fir[i].out[j] = 0.0f;
		}
		
		// ステートバッファは numTaps + blockSize - 1 のサイズが必要
		for(uint8_t j = 0; j < (NUM_TAPS + BLOCK_SIZE - 1); j++) {
			fir[i].pState[j] = 0.0f;
		}
		
		// ARM DSP FIRインスタンス初期化
		// 最後のパラメータ(blockSize)は実際に処理する際のサンプル数のための参考値
		// 実際の処理時に異なるblockSizeを指定可能
		arm_fir_init_f32(&fir_instance[i], NUM_TAPS, (float32_t*)pCoeffs, fir[i].pState, BLOCK_SIZE);
	}
	
	fir_initialized = 1;
}

void FIR_set(float data[3]) {
	// 初期化チェック
	if (!fir_initialized) {
		FIR_init();
	}
	
	// データ有効性チェック
	for(uint8_t axis = 0; axis < 3; axis++) {
		if (!isfinite(data[axis])) {
			return;  // 無効なデータの場合は処理しない
		}
	}
	
	// 観測値を保存する（メモリコピーを使用してより効率的に）
	for(uint8_t axis = 0; axis < 3; axis++) {
		// 配列を左にシフト
		for(uint8_t i = 0; i < BLOCK_SIZE - 1; i++){
			fir[axis].in[i] = fir[axis].in[i + 1];
		}
		// 新しいデータを最後に追加
		fir[axis].in[BLOCK_SIZE - 1] = data[axis];
	}
}

void FIR_calc(float data[3]) {
	// 初期化チェック
	if (!fir_initialized) {
		FIR_init();
	}
	
	// 保存されたデータにFIRをかけ、引数に出力する
	for (uint8_t i = 0; i < 3; i++){
		// ARM DSP関数を使用してFIR処理
		// ブロックサイズは実際に処理するサンプル数を指定
		arm_fir_f32(&fir_instance[i], fir[i].in, fir[i].out, BLOCK_SIZE);
		
		// 出力値の有効性チェック
		if (!isfinite(fir[i].out[BLOCK_SIZE - 1])) {
			data[i] = 0.0f;  // 無効な場合はゼロを出力
		} else {
			data[i] = fir[i].out[BLOCK_SIZE - 1];
		}
	}
}

// FIRフィルタリセット
void FIR_reset(void) {
	if (fir_initialized) {
		FIR_init();  // 再初期化
	}
}

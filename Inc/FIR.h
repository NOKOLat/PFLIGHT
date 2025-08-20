/*
 * FIR.h
 *
 *  Created on: Feb 21, 2025
 *      Author: takut
 */

#ifndef INC_FIR_H_
#define INC_FIR_H_

#include "main.h"

#define NUM_TAPS 8
#define BLOCK_SIZE 32

// FIR構造体定義
typedef struct {
	float in[BLOCK_SIZE];
	float out[BLOCK_SIZE];
	float pState[NUM_TAPS + BLOCK_SIZE - 1];
} FIR;

// 外部変数宣言
extern FIR fir[3];
extern const float pCoeffs[NUM_TAPS];

// 関数宣言
void FIR_init(void);                                    // FIRフィルタ初期化
void FIR_set(float data[3]);                           // データを入力バッファに追加
void FIR_calc(float data[3]);                          // FIRフィルタ処理実行
void FIR_process_single(float input[3], float output[3]); // サンプル処理
void FIR_reset(void);                                  // フィルタリセット

/*
 * 使用例:
 * 1. FIR_init();          // 初期化（自動で呼ばれるが明示的に呼んでも良い）
 * 
 * バッファ方式（推奨）:
 * 2. FIR_set(sensor_data); // センサーデータを追加
 * 3. FIR_calc(filtered_data); // フィルタ処理して結果を取得
 * 
 * 単一サンプル方式:
 * 2. FIR_process_single(input, output); // リアルタイム処理
 */

#endif /* INC_FIR_H_ */

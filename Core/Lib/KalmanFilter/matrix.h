/*
 * matrix.h
 *
 *  Created on: Aug 15, 2025
 *      Author: takut
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 行列最大サイズ
#ifndef MATRIX_MAX_SIZE
#define MATRIX_MAX_SIZE 5
#endif

typedef struct {
    float*  data;  // 行優先の1次元配列
    uint8_t rows;  // 行数（1..MATRIX_MAX_SIZE）
    uint8_t cols;  // 列数（1..MATRIX_MAX_SIZE）
} MATRIX;

// 行列初期化関数: 与えられた MATRIX のデータ領域を 0 クリアします（サイズは MATRIX に内包）
void MatInit(MATRIX* mat);

// 行列演算API（実装はmatrix.cpp）
// op: '+' 加算, '-' 減算, '*' 乗算, 't' 転置, 'i' 逆行列
// 失敗時はCをNaNで埋めてreturnします
void MatCalc(const MATRIX* A, const MATRIX* B, MATRIX* C, char op);

// 結果行列にNaNが含まれるかを確認（1=含む / 0=含まない）
uint8_t MatHasNaN(const MATRIX* C);

#ifdef __cplusplus
}
#endif

#endif /* INC_MATRIX_H_ */

/*
 * matrix.h
 *
 *  Created on: Aug 15, 2025
 *      Author: takut
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_

#include <cstdint>

typedef struct {
    float* data;
    uint8_t rows;
    uint8_t cols;
} MATRIX;


// 行列初期化関数: 与えられた MATRIX のデータ領域を 0 クリアします（サイズは MATRIX に内包）
void MatInit(MATRIX* mat);

// 行列演算API（実装はmatrix.cpp）
void MatCalc(const MATRIX* A, const MATRIX* B, MATRIX* C, char op);

// 結果行列にNaNが含まれるかを確認（1=含む / 0=含まない）
uint8_t MatHasNaN(const MATRIX* C);


#endif /* INC_MATRIX_H_ */

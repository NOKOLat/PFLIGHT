
/*
 * matrix.cpp
 *
 *  Created on: Aug 15, 2025
 *      Author: takut
 */
#include "matrix.h"
#include "arm_math.h" // 行列演算ライブラリ（CMSIS-DSP）
#include <math.h>

// 配列を0クリアしMATRIX型を初期化（1次元配列対応）
void MatInit(MATRIX* mat) {
	if (!mat || !mat->data || mat->cols == 0 || mat->rows == 0) {
		return;
	}
	uint16_t n = (uint16_t)mat->rows * (uint16_t)mat->cols;
	for (uint16_t i = 0; i < n; ++i) {
		mat->data[i] = 0.0f;
	}
}

// 内部関数: Cの全要素をNaNで埋める
void MatSetNaN(MATRIX* C) {
	if (!C || !C->data) return;
	uint16_t n = (uint16_t)C->rows * (uint16_t)C->cols;
	for (uint16_t i = 0; i < n; ++i) {
		C->data[i] = NAN;
	}
}

//arm_math.hを使用して行列演算を実行
void MatCalc(const MATRIX* A, const MATRIX* B, MATRIX* C, char op) {
	if (!A || !C) {
		if (C) { MatSetNaN(C); }
		return;
	}

	arm_status st = ARM_MATH_SUCCESS;
	arm_matrix_instance_f32 mat_A, mat_B, mat_C;
	arm_mat_init_f32(&mat_A, A->rows, A->cols, A->data);
	arm_mat_init_f32(&mat_C, C->rows, C->cols, C->data);

	switch (op) {
		case '+':
			// C = A + B
			if (!B) { MatSetNaN(C); break; }
			if (A->rows == B->rows && A->cols == B->cols && C->rows == A->rows && C->cols == A->cols) {
				arm_mat_init_f32(&mat_B, B->rows, B->cols, B->data);

				st = arm_mat_add_f32(&mat_A, &mat_B, &mat_C);

				if (st == ARM_MATH_SUCCESS) {
                    break;
                }
			}
			MatSetNaN(C); 
            break;

		case '-':
			// C = A - B
			if (!B) { MatSetNaN(C); break; }
			if (A->rows == B->rows && A->cols == B->cols && C->rows == A->rows && C->cols == A->cols) {
				arm_mat_init_f32(&mat_B, B->rows, B->cols, B->data);

				st = arm_mat_sub_f32(&mat_A, &mat_B, &mat_C);

				if (st == ARM_MATH_SUCCESS) {
                    break;
                }
			}
			MatSetNaN(C); 
            break;

		case '*':
			// C = A * B
			if (!B) { MatSetNaN(C); break; }
			if (A->cols == B->rows && C->rows == A->rows && C->cols == B->cols) {
				arm_mat_init_f32(&mat_B, B->rows, B->cols, B->data);

				st = arm_mat_mult_f32(&mat_A, &mat_B, &mat_C);

				if (st == ARM_MATH_SUCCESS) {
					break;
				}
			}
			MatSetNaN(C);
			break;

		case 't':
			// C = A^T
			if (C->rows == A->cols && C->cols == A->rows) {
				st = arm_mat_trans_f32(&mat_A, &mat_C);

				if (st == ARM_MATH_SUCCESS) {
					break;
				}
			}
			MatSetNaN(C);
            break;

		case 'i':
			// C = A^-1
			if (C->rows == A->rows && C->cols == A->cols && A->rows == A->cols) {
				st = arm_mat_inverse_f32(&mat_A, &mat_C);
				if (st == ARM_MATH_SUCCESS) {
					break;
				}
			}
			MatSetNaN(C);
            break;

		default:
			MatSetNaN(C);
            break;
	}
}

uint8_t MatHasNaN(const MATRIX* C) {
	if (!C || !C->data) {
        return 1;
	}
	uint16_t n = (uint16_t)C->rows * (uint16_t)C->cols;
	for (uint16_t i = 0; i < n; ++i) {
		if (isnan(C->data[i])) {
            return 1;
		}
	}

    return 0;
}






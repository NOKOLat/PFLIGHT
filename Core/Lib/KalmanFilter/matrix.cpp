/*
 * matrix.cpp
 *
 *  Created on: Aug 15, 2025
 *      Author: takut
 */
#include "matrix.h"
#include <math.h>

static uint8_t MatPrecheck(const MATRIX* A, const MATRIX* B, MATRIX* C);
static uint8_t MatValid(const MATRIX* M);
static void MatSetNaN(MATRIX* C);

static void MatAdd(const MATRIX* A, const MATRIX* B, MATRIX* C);
static void MatSub(const MATRIX* A, const MATRIX* B, MATRIX* C);
static void MatMul(const MATRIX* A, const MATRIX* B, MATRIX* C);
static void MatTrans(const MATRIX* A, MATRIX* C);
static void MatInv(const MATRIX* A, MATRIX* C);


// 配列を0クリアしMATRIX型を初期化
void MatInit(MATRIX* mat) {
    if (!MatValid(mat)) return;
    uint16_t n = (uint16_t)mat->rows * (uint16_t)mat->cols;
    for (uint16_t i = 0; i < n; ++i) {
        mat->data[i] = 0.0f;
    }
}

// 行列演算
void MatCalc(const MATRIX* A, const MATRIX* B, MATRIX* C, char op) {
    if (!MatPrecheck(A, B, C)) {
        return; // 共通の前提チェックを呼び出し
    }

    switch (op) {
        case '+': MatAdd(A, B, C); break;;
        case '-': MatSub(A, B, C); break;
        case '*': MatMul(A, B, C); break;
        case 't': MatTrans(A, C);  break;
        case 'i': MatInv(A, C);    break;
        default:  MatSetNaN(C);    break;
    }
}

// 内部演算: C = A + B
static void MatAdd(const MATRIX* A, const MATRIX* B, MATRIX* C) {
    if (!(A->rows == B->rows && A->cols == B->cols &&
          C->rows == A->rows && C->cols == A->cols)) { MatSetNaN(C); return; }
    uint16_t n = (uint16_t)A->rows * (uint16_t)A->cols;
    for (uint16_t i = 0; i < n; ++i) C->data[i] = A->data[i] + B->data[i];
}

// 内部演算: C = A - B
static void MatSub(const MATRIX* A, const MATRIX* B, MATRIX* C) {
    if (!(A->rows == B->rows && A->cols == B->cols &&
          C->rows == A->rows && C->cols == A->cols)) { MatSetNaN(C); return; }
    uint16_t n = (uint16_t)A->rows * (uint16_t)A->cols;
    for (uint16_t i = 0; i < n; ++i) C->data[i] = A->data[i] - B->data[i];
}

// 内部演算: C = A * B
static void MatMul(const MATRIX* A, const MATRIX* B, MATRIX* C) {
    if (!(A->cols == B->rows && C->rows == A->rows && C->cols == B->cols)) { MatSetNaN(C); return; }
    const uint8_t R = C->rows, K = A->cols, T = C->cols;
    float tmp[MATRIX_MAX_SIZE * MATRIX_MAX_SIZE];
    float* out = C->data;
    const uint8_t alias = (C->data == A->data) || (C->data == B->data);
    if (alias) out = tmp;
    for (uint8_t i = 0; i < R; ++i) {
        for (uint8_t j = 0; j < T; ++j) {
            float s = 0.0f;
            for (uint8_t k = 0; k < K; ++k) s += A->data[i * K + k] * B->data[k * T + j];
            out[i * T + j] = s;
        }
    }
    if (alias) {
        uint16_t n = (uint16_t)R * (uint16_t)T;
        for (uint16_t i = 0; i < n; ++i) C->data[i] = out[i];
    }
}

// 内部演算: C = A^T
static void MatTrans(const MATRIX* A, MATRIX* C) {
    if (!(C->rows == A->cols && C->cols == A->rows)) { MatSetNaN(C); return; }
    const uint8_t R = A->rows, T = A->cols;
    const uint8_t alias = (C->data == A->data);
    float tmp[MATRIX_MAX_SIZE * MATRIX_MAX_SIZE];
    float* out = C->data;
    if (alias) out = tmp;
    for (uint8_t i = 0; i < R; ++i)
        for (uint8_t j = 0; j < T; ++j)
            out[j * C->cols + i] = A->data[i * A->cols + j];
    if (alias) {
        uint16_t n = (uint16_t)C->rows * (uint16_t)C->cols;
        for (uint16_t i = 0; i < n; ++i) C->data[i] = out[i];
    }
}

// 内部演算: C = A^-1（ガウス・ジョルダン、部分ピボット付き）
static void MatInv(const MATRIX* A, MATRIX* C) {
    if (!(A->rows == A->cols && C->rows == A->rows && C->cols == A->cols)) { MatSetNaN(C); return; }
    const uint8_t n = A->rows;
    float aug[MATRIX_MAX_SIZE * (2 * MATRIX_MAX_SIZE)]; // [A|I]
    // 構築
    for (uint8_t i = 0; i < n; ++i) {
        for (uint8_t j = 0; j < n; ++j) {
            aug[i * (2 * n) + j]       = A->data[i * n + j];
            aug[i * (2 * n) + (j + n)] = (i == j) ? 1.0f : 0.0f;
        }
    }
    const float eps = 1e-8f;
    // 変形
    for (uint8_t col = 0; col < n; ++col) {
        uint8_t piv = col; float maxAbs = fabsf(aug[col * (2 * n) + col]);
        for (uint8_t r = col + 1; r < n; ++r) {
            float v = fabsf(aug[r * (2 * n) + col]);
            if (v > maxAbs) { maxAbs = v; piv = r; }
        }
        if (maxAbs < eps) { MatSetNaN(C); return; }
        if (piv != col) {
            for (uint8_t j = 0; j < 2 * n; ++j) {
                float t = aug[col * (2 * n) + j];
                aug[col * (2 * n) + j] = aug[piv * (2 * n) + j];
                aug[piv * (2 * n) + j] = t;
            }
        }
        float pivot = aug[col * (2 * n) + col];
        for (uint8_t j = 0; j < 2 * n; ++j) aug[col * (2 * n) + j] /= pivot;
        for (uint8_t r = 0; r < n; ++r) if (r != col) {
            float f = aug[r * (2 * n) + col]; if (f == 0.0f) continue;
            for (uint8_t j = 0; j < 2 * n; ++j) aug[r * (2 * n) + j] -= f * aug[col * (2 * n) + j];
        }
    }
    // 右側をCへ
    for (uint8_t i = 0; i < n; ++i)
        for (uint8_t j = 0; j < n; ++j)
            C->data[i * n + j] = aug[i * (2 * n) + (j + n)];
}


// 内部関数: Cの全要素をNaNで埋める
static void MatSetNaN(MATRIX* C) {
    if (!C || !C->data) {
        return;
    }
    uint16_t n = (uint16_t)C->rows * (uint16_t)C->cols;
    for (uint16_t i = 0; i < n; ++i) {
        C->data[i] = NAN;
    }
}

// 内部関数: 行列の基本妥当性チェック
static uint8_t MatValid(const MATRIX* M) {
    return (M && M->data && M->rows > 0 && M->cols > 0);
}

// 共通の前提チェック（失敗時はCをNaN埋めして1を返す）
static uint8_t MatPrecheck(const MATRIX* A, const MATRIX* B, MATRIX* C) {
    if (!C || !C->data) return 1;
    if (MatValid(A)) { MatSetNaN(C); return 1; }
    if (C->rows == 0 || C->cols == 0) { MatSetNaN(C); return 1; }
    if (A->rows > MATRIX_MAX_SIZE || A->cols > MATRIX_MAX_SIZE ||
        C->rows > MATRIX_MAX_SIZE || C->cols > MATRIX_MAX_SIZE) { MatSetNaN(C); return 1; }
    if (B && (B->rows > MATRIX_MAX_SIZE || B->cols > MATRIX_MAX_SIZE)) { MatSetNaN(C); return 1; }
    return 0;
}



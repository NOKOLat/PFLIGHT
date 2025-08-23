# 2025_PFLIGHT_KalmanFilter
### AIによる自動生成

# カルマンフィルターモジュール

## 概要

このモジュールは、任意次元のカルマンフィルターを簡単に利用できるC++クラス [`Kalman`](Inc/kalman.h) を提供します。  
状態遷移行列や観測行列、ノイズパラメータなどを柔軟に設定でき、組み込み用途にも適した設計です。

## 主な機能

- 任意次元のカルマンフィルター（最大4次元まで拡張可能）
- 行列演算は [`matrix.h`](Inc/matrix.h) を利用
- 状態遷移行列・観測行列・ノイズパラメータを動的に設定可能
- 1次元～多次元の状態推定に対応

## クラス構成

- [`Kalman`](Inc/kalman.h)
  - `void Init(uint8_t state_size, uint8_t obs_size);`  
    状態数・観測数を指定して初期化
  - `void Update();`  
    1ステップのカルマンフィルター計算を実行
  - `void GetData(float* out_states);`  
    推定結果を取得
  - 各種パラメータ（行列・ノイズ）はpublicメンバとして直接アクセス可能

## 使い方

### 1. インスタンス生成・初期化

```cpp
Kalman kalman;
kalman.Init(2, 1); // 状態2次元・観測1次元
```

### 2. 行列・ノイズパラメータの設定
```cpp
// 状態遷移行列F, 観測行列H, ノイズ
kalman.system_matrix[0] = 1.0f; kalman.system_matrix[1] = dt;
kalman.system_matrix[2] = 0.0f; kalman.system_matrix[3] = 1.0f;
kalman.observation_matrix[0] = 1.0f; kalman.observation_matrix[1] = 0.0f;
kalman.prediction_noise = 0.01f;
kalman.observation_noise = 1.0f;
```

### 3. 観測値・予測値の設定
```cpp
kalman.observation[0] = 測定値;
kalman.prediction[0] = ...; // 予測値（例: 前回値 + 速度×dt）
kalman.prediction[1] = ...;
```
### 4. フィルタ更新・結果取得
```cpp
kalman.Update();
float result[2];
kalman.GetData(result);
// result[0]: 推定値, result[1]: 推定速度 など
```

### 使用例：高度推定（Src/altitude.cppより）
```cpp
// 状態: [高度, 速度]、観測: [高度]
kalman.system_matrix[0] = 1.0f; kalman.system_matrix[1] = dt;
kalman.system_matrix[2] = 0.0f; kalman.system_matrix[3] = 1.0f;
kalman.observation_matrix[0] = 1.0f; kalman.observation_matrix[1] = 0.0f;

kalman.prediction[0] = estimated_altitude + estimated_velocity * dt + 0.5f * estimated_accel * dt * dt;
kalman.prediction[1] = estimated_velocity + estimated_accel * dt;

kalman.prediction_noise = 0.01f;
kalman.observation_noise = 1.0f;

if (pressure_hPa > 0.0f && !isnan(pressure_hPa)) {
    float altitude_measurement = PressureToAltitude(pressure_hPa);
    kalman.observation[0] = altitude_measurement;
    kalman.Update();
    float result[2];
    kalman.GetData(result);
    estimated_altitude = result[0];
    estimated_velocity = result[1];
} else {
    estimated_altitude = kalman.prediction[0];
    estimated_velocity = kalman.prediction[1];
}
```




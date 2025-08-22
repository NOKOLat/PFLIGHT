# 2025_PFLIGHT_PID
2025年度の自作FC『PFLIGHT』のPIDライブラリです。汎用的なプログラムなので、マルコプのFC以外にも利用できます。

## ファイル構造

- `PID.h` 
- `PID.cpp`

## 1軸の使用例（呼び出し/計算/出力取得）

```cpp
#include "PID.h"
#include <cstdio>

int main() {
    // 1) コンストラクタで初期化（P, I, D, 制御周期[s]）
    PID pid(0.8f, 0.1f, 0.05f, 0.01f);

    // 2) 上限値の設定
    pid.setLimit(10.0f, 5.0f); // i_max, d_max

    float target = 1.0f;        // 目標値
    float process_value = 0.0f; // 現在のプロセス値

    for (int k = 0; k < 100; k++) {
        // 3) 計算（target と process_value を渡す）
        pid.calc(target, process_value);

        // 4) 出力取得（制御量）
        float u = pid.getData();

        //シミュレーションの仮のモデル
        process_value += 0.02f * (u - process_value);

        std::printf("k=%d, pv=%.3f, u=%.3f\n", k, process_value, u);
    }

    return 0;
}
```

- 既に生成済みのインスタンスへ後からパラメータを設定する場合は、`setGain(p,i,d)`, `setTime(dt)`, `setLimit(i_max,d_max)` を使用します。

## コンストラクタの仕様

- `PID()`
  - 既定値で生成。`setGain`, `setTime`, `setLimit` を後で呼ぶ前提。
- `PID(float p_gain, float i_gain, float d_gain, float time)`
  - ゲインと制御周期を設定。上限は `setLimit(1000.0f, 1000.0f)` が適用されます。
- `PID(float p_gain, float i_gain, float d_gain, float d_max, float i_max, float time)`
  - ゲイン、制御周期、I/D の上限をすべて指定。

## 値の範囲外処理

- `setGain(p_gain, i_gain, d_gain)`
  - それぞれが「有限かつ 0 より大きい」ときのみ採用。
  - 条件を満たさないときは 0 に設定

- `setTime(time)`
  - `time` が「有限かつ `time_min` より大きい」時のみ採用。
  - それ以外は `time = time_min` に設定（既定: `time_min = 1e-6f`）

- `setLimit(i_max, d_max)`
  - それぞれが「有限かつ 0 以上」のときのみ採用。
  - 条件を満たさないときは 0 に設定。

## 積分方法（I項の更新）

- 初期2サンプルは台形近似
- 3サンプル目以降は3点のシンプソン公式で更新。
  - `integral += (time/3) * (e_{k-2} + 4*e_{k-1} + e_k)`
　

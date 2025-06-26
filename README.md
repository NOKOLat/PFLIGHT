#  PFLIGHT V1.30(beta)

## UpDate

- USER_Setting.hppを追加
  - すべての設定項目をこのファイルから触れるようにした
  - 将来的な通信ツールの実装用

- モーターを回し始める時の条件にthrottleの値を追加

- PIDとPWM周りの関数のリファクタリング

## Setup

- クロックとタイマーと割り込みの設定をしてください

### Clock

- mainclock 216mhz
- Timer 108Mhz（両方とも同じ）

### UART5 (SBUS)
100kbps, even, 9bits, Pinlevel反転

### TIM 1(ch 1234) TIM12(ch 12)

- Pre 107
- Counter 2499
- 216Mhz
  
### TIM6
- 割り込みの有効化
- Pre 107
- Counter 2499

## PID 

- PIDの設定方法です
- 角度制御から角速度の目標値を計算して、それを角速度でPIDしています

### 設定方法

- PIDUSER.hppを開く
- PidSetup()内の各セットアップ関数内の引数を変更する

```cpp
PidSetup(float kp, float ki, float kd, float time)
```


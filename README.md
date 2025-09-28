# PFLIGHT v2.11

> [!WARNING]
> PIDの設定値が共有できていない可能性があるため、
> 飛行前には`UserSetting/PIDSetting.hpp`で値を確認してください

マルチコプターのFCコードです

練習飛行を目的としているので、いくつかの機能が消されて小さなプログラムになっています

情報はこちらにも書いてあります
[PFLIGHT_Wiki](https://github.com/NOKOLat/PFLIGHT/wiki)

## 1. 使用について

- 2025年9月30日までは、開発チーム以外の一切の使用を禁止します
- 2025年10月1日以降は、下記のライセンスしたがって利用してください

## 2. ライセンスについて

- このプロジェクトで作成したファイルはすべてMITライセンスで公開しています

基本的に自由に使用できますが、何が起きても自己責任になります

ただし、以下は外部のコードを使用しているため、作成者が設定したライブラリに依存します

- STM32CubeIDEによる生成ファイル(各ファイルやST公式の記述を参照してください)
- [Core/Lib/MadgwickAHRS](https://github.com/arduino-libraries/MadgwickAHRS) 
- [Core/Lib/SBUS](https://github.com/NOKOLat/SBUS)
- [Core/Lib/RingBuffer](https://github.com/NOKOLat/Ring-Buffer)


## 3. 動作環境

- MCU: STM32F732RET6
- IMU: ICM42688P
- 無線通信: SBUS(10channel)
- 開発環境: STM32CubeIDE v1.18.1

## 4. 使い方

1. このリポジトリをクローン

2. STM32CubeIDEで開き、CodeGenerateを行う

3. 使用する基板に書き込みを行う

- ```Core/Inc/UserSetting```でPIDやモーターのPWM値の調節ができます

## 5. 飛ばし方

- SBUSのChannel6(arm)を常時オンにできるスイッチに割り当て
- SBUSのChannel5(fly)を一時的にオンになるスイッチに割り当ててください

- Channel5によってArm状態に移行し、Channel6によって飛行状態に移行できます
- プロポのスイッチ切断 or Channel5をオフにすることでDisArmします
```
Core/
├─ Inc/                    # ヘッダー群（STM32Cube生成ヘッダは省略）
│  ├─ FlightManager.h      # 状態管理クラスの定義
│  ├─ wrapper.hpp          # C++ 用ラッパー（Cインターフェースの薄いラッパー）
│  ├─ FlightData/          # データ構造体
│  │  ├─ ControlData.hpp   # 制御出力・PWM 値を保持
│  │  ├─ SbusData.hpp      # SBUS 入力データ
│  │  └─ SensorData.hpp    # IMU 等のセンサデータ定義
│  ├─ State/
│  │  └─ Headers/
│  │     └─ FlightStates.h  # 各フライト状態列挙など
│  ├─ State/Interface/
│  │  └─ FlightStateInterface.h # 状態クラスの共通インターフェース
│  ├─ UserSetting/         # ユーザーが変更する設定値
│  │  ├─ MotorSetting.hpp  # モーター設定（回転方向・PWMレンジ等）
│  │  ├─ LEDSetting.hpp    # LED 表示設定
│  │  ├─ ImuSetting.hpp    # IMU 設定
│  │  └─ PIDSetting.hpp    # PID 初期値・ゲイン
│  └─ Utils/               # 小さなユーティリティ・ラッパー
│     ├─ ICM42688P_SPI_Util.hpp # ICM42688P の SPI 用ユーティリティ
│     ├─ LED.hpp           # LED 操作用ラッパークラス
│     ├─ PWM.hpp           # PWM 操作用ラッパークラス
│     └─ SbusDecoder.hpp   # SBUS デコードロジック

├─ Lib/                    # 外部ライブラリ（プロジェクト内に同梱）
│  ├─ ICM42688P/           # ICM42688P IMU ドライバ
│  │  ├─ ICM42688P.cpp
│  │  ├─ ICM42688P.h
│  │  ├─ ICM42688P_HAL_I2C.cpp/.h
│  │  ├─ ICM42688P_HAL_SPI.cpp/.h
│  │  └─ ICM42688P_Wire_I2C.*
│  ├─ KalmanFilter/        # （オプション）カルマンフィルタ（未使用）
│  │  ├─ kalman.cpp/.h
│  │  └─ matrix.cpp/.h
│  ├─ MadgwickAHRS/        # Madgwick AHRS（姿勢推定）
│  │  └─ src/MadgwickAHRS.cpp/.h
│  ├─ PID/                 # PID コントローラ実装
│  │  ├─ PID.cpp
│  │  └─ PID.h
│  ├─ ringBuffer/          # リングバッファ
│  │  └─ ringBuffer.h
│  └─ SBUS/                # SBUS 入力ライブラリ
│     ├─ sbus.cpp
│     └─ sbus.h

└─ Src/                    # 実装ソース
   ├─ main.cpp                    # エントリーポイント
   ├─ FlightManager.cpp           # 状態管理の実装
   ├─ wrapper.cpp                 # C ラッパー実装
   ├─ State/                      # 各状態の実装
   │  ├─ InitState.cpp
   │  ├─ PreArmingState.cpp
   │  ├─ PreFlightState.cpp
   │  ├─ FlyingState.cpp
   │  ├─ AutoFlyState.cpp         # 未実装（雛形あり）
   │  ├─ DisarmingState.cpp
   │  ├─ EmergencyControlState.cpp# 未実装（雛形あり）
   │  └─ FailSafeState.cpp
   └─ Utils/
      ├─ ICM42688P_SPI_Util.cpp
      └─ PWM.cpp
```
│     ├─ sbus.cpp
│     └─ sbus.h
│
└─ Src/
   ├─ main.cpp                      // エントリーポイント
   ├─ FlightManager.cpp             // 状態管理クラス
   ├─ wrapper.cpp
   ├─ State/
   │  ├─ InitState.cpp
   │  ├─ PreArmingState.cpp
   │  ├─ PreFlightState.cpp
   │  ├─ FlyingState.cpp
   │  ├─ AutoFlyState.cpp           // 未実装
   │  ├─ DisarmingState.cpp
   │  ├─ EmergencyControlState.cpp  // 未実装
   │  └─ FailSafeState.cpp
   └─ Utils/
      ├─ ICM42688P_SPI_Util.cpp
      └─ PWM.cpp              
```

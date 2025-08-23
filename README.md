# PFLIGHT v2.10.2

2025年度の部内大会で使用するはずのコード

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

## 6. デバックとLEDについて

- STLinkなどでFCとPCを接続することで、エラー出力や飛行状態の確認を行うことができます

- 通常時はLEDを確認することで、現在がどの状態かを判定することができます

| LED        | 状態      
|--------------|------------------------|
| 赤点灯 | 初期化成功     |
| 黄点灯 | Arm成功     |
| 緑点灯 | 飛行中     |
| 全点滅 | FailSafe     |

## 7. ファイル構成

Coreの実装はこのようになっています(STM32CubeIDEによる生成ファイルは省略)

Stateパターンをベースとした状態遷移を行っています

```
Core/
├─ Inc/          
│  ├─ FlightManager.h       // 状態管理クラスのヘッダー                      
│  ├─ wrapper.hpp           // C++ 用ラッパー
│  ├─ FlightData/           // データ用の構造体
│  │  ├─ ControlData.hpp    // 制御出力とPWM値
│  │  ├─ SbusData.hpp       // SBUS
│  │  └─ SensorData.hpp     // センサデータ
│  ├─ State/                
│  │  └─ Headers/
│  │     └─ FlightStates.h   // 各状態の定義
│  ├─ State/Interface/
│  │  └─ FlightStateInterface.h // 状態クラスのインターフェース
│  ├─ UserSetting/          
│  │  ├─ MotorSetting.hpp   // モーター設定
│  │  └─ PIDSetting.hpp     // PID設定
│  └─ Utils/               
│     ├─ ICM42688P_SPI_Util.hpp // ICM42688PSPI用ユーティリティ
│     ├─ LED.hpp            // LEDラッパー
│     ├─ PWM.hpp            // PWMラッパー
│     └─ SbusDecoder.hpp    // SBUSデコード
│
├─ Lib/
│  ├─ ICM42688P/            // IMU ライブラリ
│  │  ├─ ICM42688P.cpp
│  │  ├─ ICM42688P.h
│  │  ├─ ICM42688P_HAL_I2C.cpp/.h
│  │  ├─ ICM42688P_HAL_SPI.cpp/.h
│  │  └─ ICM42688P_Wire_I2C.* 
│  ├─ KalmanFilter/         // カルマンフィルタ（現状未使用）
│  │  ├─ kalman.cpp/.h
│  │  └─ matrix.cpp/.h
│  ├─ MadgwickAHRS/         // Madgwick AHRS ライブラリソース
│  │  └─ src/MadgwickAHRS.cpp/.h
│  ├─ PID/                  // PID ライブラリ
│  │  ├─ PID.cpp
│  │  └─ PID.h
│  ├─ ringBuffer/           // リングバッファライブラリ
│  │  └─ ringBuffer.h
│  └─ SBUS/                 // SBUS ライブラリ
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

# PFLIGHT 

2025年度に作成した、4発マルチコプターのFCプログラムです

現在開発中につき、同じチームの人以外の使用を禁止します

（大会で使用する + 安全上の欠陥が高確率で存在しているため）

## 使い方

LEDやTeraTermでのシリアル受信によって現在のFCの状態がわかります

| LED        | 状態      
|--------------|------------------------|
| 赤点灯 | 初期化成功     |
| 黄点灯 | Arm成功     |
| 緑点灯 | 飛行中     |
| 全点滅 | FailSafe     |


## 必要なもの

| 役割         | 使用しているもの        | 代替方法              |
|--------------|------------------------|-----------------------|
| MCU（制御用マイコン） | STM32F732RET6         | 他のSTM32シリーズ、64KB以上推奨|
| IMU（慣性計測ユニット） | ICM42688P              | MPU6050、ICM20948など |

## ファイル構成

STM32CubeIDEの標準的なプロジェクト構成を取っています
使用する際には```Inc/Src```ファイル内のファイルを```Project/Core/Inc``` or ```Project/Core/Src```に入れてください

```
PFLIGHT/
├── PFLIGHT.ioc           　　　　　# STM32CubeIDEプロジェクト設定ファイル
├── Readme.md                       # プロジェクト説明書
├── STM32F732RETX_FLASH.ld          # Flashメモリリンカスクリプト
├── STM32F732RETX_RAM.ld            # RAMリンカスクリプト
├── Core/                           # メインソースコード
│   ├── Inc/                        # ヘッダファイル
│   │   ├── main.h                  # メイン関数のヘッダ
│   │   ├── flight_manager.hpp      # フライト管理システム
│   │   ├── flight_data.hpp         # フライトデータ構造体
│   │   ├── IMU.hpp                 # IMUセンサー制御
│   │   ├── ICM42688P.h             # ICM42688P IMUセンサードライバ
│   │   ├── ICM42688P_HAL_SPI.h     # ICM42688P SPI通信
│   │   ├── MadgwickAHRS.h          # Madgwick姿勢推定アルゴリズム
│   │   ├── MadgwickAHRS_USER.hpp   # Madgwick姿勢推定カスタム
│   │   ├── PID.h                   # PID制御
│   │   ├── PID_USER.hpp            # PIDコントローラカスタム
│   │   ├── PWM.hpp                 # PWM信号制御
│   │   ├── LED.hpp                 # LED制御
│   │   ├── sbus.h                  # SBUS通信プロトコル
│   │   ├── ringBuffer.h            # リングバッファ
│   │   ├── wrapper.hpp             # C/C++ラッパー関数
│   │   ├── adc.h                   # ADC制御
│   │   ├── dma.h                   # DMA制御
│   │   ├── gpio.h                  # GPIO制御
│   │   ├── spi.h                   # SPI通信
│   │   ├── tim.h                   # タイマー制御
│   │   ├── usart.h                 # UART通信
│   │   ├── stm32f7xx_hal_conf.h    # HALライブラリ設定
│   │   └── stm32f7xx_it.h          # 割り込みハンドラ
│   ├── Src/                        # ソースファイル
│   │   ├── main.c                  # メイン関数
│   │   ├── flight_namager.cpp      # フライト管理システム実装
│   │   ├── ICM42688P.cpp           # ICM42688P IMUドライバ実装
│   │   ├── ICM42688P_HAL_SPI.cpp   # ICM42688P SPI通信実装
│   │   ├── MadgwickAHRS.cpp        # Madgwick姿勢推定実装
│   │   ├── PID.cpp                 # PID制御実装
│   │   ├── PWM.cpp                 # PWM制御実装
│   │   ├── sbus.cpp                # SBUS通信実装
│   │   ├── wrapper.cpp             # C/C++ラッパー実装
│   │   ├── adc.c                   # ADC制御実装
│   │   ├── dma.c                   # DMA制御実装
│   │   ├── gpio.c                  # GPIO制御実装
│   │   ├── spi.c                   # SPI通信実装
│   │   ├── tim.c                   # タイマー制御実装
│   │   ├── usart.c                 # UART通信実装
│   │   ├── stm32f7xx_hal_msp.c     # HAL MSP設定
│   │   ├── stm32f7xx_it.c          # 割り込みハンドラ実装
│   │   └── system_stm32f7xx.c      # システム初期化
│   └── Startup/                    # スタートアップファイル
│       └── startup_stm32f732retx.s # アセンブリスタートアップ
├── Debug/                          # デバッグビルド出力
├── Drivers/                        # STM32 HALドライバ
│   ├── CMSIS/                      # CMSIS標準ライブラリ
│   └── STM32F7xx_HAL_Driver/       # STM32F7 HALドライバ
└── .settings/                      # IDE設定ファイル
```




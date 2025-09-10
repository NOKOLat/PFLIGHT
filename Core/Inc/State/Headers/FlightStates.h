#pragma once

#include "State/Interface/FlightStateInterface.h"
#include "FlightManager.h"

#include <memory>

#include "adc.h"
#include "Utils/PWM.hpp"
#include "Utils/LED.hpp"

// 前方宣言
class FlightManager;

// 初期化状態
class InitState : public FlightStateInterface {

    public:
        void update(FlightManager& manager) override;
        void enter(FlightManager& manager) override;
        const char* getStateName() const override { return "InitState"; }
};

// arm
class PreArmingState : public FlightStateInterface {

    public:
        void update(FlightManager& manager) override;
        const char* getStateName() const override { return "PreArming"; }
};

// PreFlight
class PreFlightState : public FlightStateInterface {

    public:
        void update(FlightManager& manager) override;

        const char* getStateName() const override { return "PreFlight"; }
};

// 飛行状態
class FlyingState : public FlightStateInterface {

    public:
        void update(FlightManager& manager) override;
        const char* getStateName() const override { return "Flying"; }
};

// Disarm状態
class DisarmingState : public FlightStateInterface {

    public:
        void update(FlightManager& manager) override;
        void enter(FlightManager& manager) override;
        void exit(FlightManager& manager) override;
        const char* getStateName() const override { return "Disarming"; }
};

// 対故障制御
class EmergencyControlState : public FlightStateInterface {

    public:
        void update(FlightManager& manager) override;
        void enter(FlightManager& manager) override;
        void exit(FlightManager& manager) override;
        const char* getStateName() const override { return "EmergencyControl"; }
};

// 自動操縦
class AutoFlyState : public FlightStateInterface {

    public:
        void update(FlightManager& manager) override;
        void enter(FlightManager& manager) override;
        void exit(FlightManager& manager) override;
        const char* getStateName() const override { return "AutoFly"; }
};

// フェイルセーフ
class FailSafeState : public FlightStateInterface {

    public:
        void update(FlightManager& manager) override;
        void enter(FlightManager& manager) override;
        const char* getStateName() const override { return "FailSafe"; }
};

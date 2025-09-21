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
    void Update(FlightManager& manager) override;
    void Enter(FlightManager& manager) override;
    const char* GetStateName() const override { return "InitState"; }
};

// arm
class PreArmingState : public FlightStateInterface {

    public:
    void Update(FlightManager& manager) override;
    const char* GetStateName() const override { return "PreArming"; }
};

// PreFlight
class PreFlightState : public FlightStateInterface {

    public:
    void Update(FlightManager& manager) override;

    const char* GetStateName() const override { return "PreFlight"; }
};

// 飛行状態
class FlyingState : public FlightStateInterface {

    public:
    void Update(FlightManager& manager) override;
    const char* GetStateName() const override { return "Flying"; }
};

// Disarm状態
class DisarmingState : public FlightStateInterface {

    public:
    void Update(FlightManager& manager) override;
    void Enter(FlightManager& manager) override;
    void Exit(FlightManager& manager) override;
    const char* GetStateName() const override { return "Disarming"; }
};

// フェイルセーフ
class FailSafeState : public FlightStateInterface {

    public:
    void Update(FlightManager& manager) override;
    void Enter(FlightManager& manager) override;
    const char* GetStateName() const override { return "FailSafe"; }
};

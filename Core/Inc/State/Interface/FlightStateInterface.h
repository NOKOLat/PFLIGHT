#ifndef INC_FLIGHTSTATE_INTERFACE_HPP_
#define INC_FLIGHTSTATE_INTERFACE_HPP_

// 前方宣言
class FlightManager;

// フライト状態のインターフェース（抽象基底クラス）
class FlightStateInterface {
public:
    virtual ~FlightStateInterface() = default;
    virtual void Update(FlightManager& manager) = 0;
    virtual void Enter(FlightManager& manager) {}
    virtual void Exit(FlightManager& manager) {}
    virtual const char* GetStateName() const = 0;
};

#endif /* INC_FLIGHTSTATE_INTERFACE_HPP_ */

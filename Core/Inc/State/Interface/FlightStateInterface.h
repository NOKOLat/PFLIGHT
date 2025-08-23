#ifndef INC_FLIGHTSTATE_INTERFACE_HPP_
#define INC_FLIGHTSTATE_INTERFACE_HPP_

// 前方宣言
class FlightManager;

// フライト状態のインターフェース（抽象基底クラス）
class FlightStateInterface {
public:
    virtual ~FlightStateInterface() = default;
    virtual void update(FlightManager& manager) = 0;
    virtual void enter(FlightManager& manager) {}
    virtual void exit(FlightManager& manager) {}
    virtual const char* getStateName() const = 0;
};

#endif /* INC_FLIGHTSTATE_INTERFACE_HPP_ */

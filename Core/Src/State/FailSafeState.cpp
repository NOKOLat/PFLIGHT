#include "State/Headers/FlightStates.h"

void FailSafeState::update(FlightManager& manager) {

	while(1){

		for(volatile uint32_t i=0; i<1000000; i++);

		redLed(PinState::toggle);
		yellowLed(PinState::toggle);
		greenLed(PinState::toggle);
	}
}

void FailSafeState::enter(FlightManager& manager) {

	PwmStop();
	manager.angle_pitch.reset();
	manager.angle_roll.reset();
	manager.rate_pitch.reset();
	manager.rate_roll.reset();
	manager.rate_yaw.reset();
}


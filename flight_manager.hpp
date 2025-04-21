#ifndef INC_FLIGHT_MANEGER_HPP_
#define INC_FLIGHT_MANEGER_HPP_

#include <cstdint>

struct FlightManager{

	uint8_t wait = 0;
	uint8_t loop_count = 0;
}armloop;

uint8_t IsWaitLoop(){

	return armloop.wait;
}

void ArmLoopReset(){

	armloop.wait = 0;
	armloop.loop_count = 0;
}

void ArmLoopSet(){

	armloop.wait = 1;
}

void ArmLoopClear(){

	armloop.wait = 0;
}


#endif

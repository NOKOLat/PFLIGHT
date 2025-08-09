#ifndef INC_SBUS_HPP_
#define INC_SBUS_HPP_

#include <array>
#include <iterator>
#include <stdint.h>

#include "ringBuffer.h"

namespace nokolat {

struct SBUS_DATA{
    std::array<uint16_t,18> data;
    bool failsafe = false;
    bool framelost = false;

    uint16_t &operator[](std::size_t __n) noexcept{
        return data[__n];
    }
    uint16_t &at(std::size_t __n){
        return data.at(__n);
    }

    SBUS_DATA(std::array<uint16_t,18> data = {})
    :data(data){}
};

class SBUS {
public:
    SBUS(){}

    auto getBufferIterator(){
        return receiveBuffer.cbegin();
    }

    uint8_t *getReceiveBufferPtr() {
        return (uint8_t*)receiveBuffer.data();
    }

    int16_t getData(const uint8_t channel){
        if(channel > 18 or channel < 1) return -1;
        if(needParse){
            parse(receiveBuffer,data);
        }

        return data[channel-1];
    }

    const SBUS_DATA &getData(){
        if(needParse){
            parse(receiveBuffer,data);
        }
        return data;
    }

    constexpr uint8_t getDataLen(){
        return length;
    }

    void requireParse(bool arg=true){
        needParse = arg;
    }

    std::array<uint8_t, 25> getRawBuffer(){
    	std::array<uint8_t, 25> res;
    	for(uint8_t n=0; n<25; n++){
    		res[n] = receiveBuffer[n];
    	}
    	return res;
    }

    inline void parse(){
        parse(receiveBuffer, data);
    }
    SBUS_DATA parse(std::array<uint8_t,25> &arg);
    SBUS_DATA parse(RingBuffer<uint8_t,25> &arg);
    void parse(std::array<uint8_t,25> &arg, SBUS_DATA &res);
    void parse(RingBuffer<uint8_t,25> &arg, SBUS_DATA &res);

    std::array<uint8_t,25> convertToRawArrey(){
    	return convertToRawArrey(data);
    }
    std::array<uint8_t,25> convertToRawArrey(SBUS_DATA &arg);

protected:
    SBUS_DATA data;
    RingBuffer<uint8_t,25> receiveBuffer;
private:
    bool needParse;

    const uint8_t length = 25;

    const uint8_t HEADER = 0x0f;
    const uint8_t FOOTER = 0x00;
    const uint8_t FOOTER2 = 0x04;
};

} /* namespace nokolat */

#endif /* INC_SBUS_HPP_ */

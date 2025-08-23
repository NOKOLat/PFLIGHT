# SBUS
## Methods
### uint8_t *getReceiveBufferPtr()
Return a pointer to the receive buffer.
### int16_t getData(const uint8_t channel)
Return a value of the specified channel given in an argument.
### const SBUS_DATA &getData()
Return a refarence for the SBUS_DATA.
### constexpr uint8_t getDataLen()
Return a length of the receive buffer.
### void requireParse(bool arg=true)
This function tells for the instance to new data has been received and require a parsing to update data.
### std::array<uint8_t, 25> getRawBuffer()
Return a raw array of the receved data.
The array is aligned to begin with the headder.
### parse
There are 5 variation of the return type and arguments.
#### void parse()
This function parse internal buffer and ready to use data.
#### SBUS_DATA parse(std::array<uint8_t,25> &arg)
#### SBUS_DATA parse(RingBuffer<uint8_t,25> &arg)
#### void parse(std::array<uint8_t,25> &arg, SBUS_DATA &res)
#### void parse(RingBuffer<uint8_t,25> &arg, SBUS_DATA &res)

### uint16_t convertSbus2PwmWidthUS(uint16_t arg)
This function convert a sbus raw value that is giben in argument to pwm pulse width (us).

## Usage for STM32 HAL library
```c++
nokolat::SBUS sbus;
nokolat::SBUS_DATA sbusData;

HAL_UART_Receive(&huart1, sbus.getReceiveBufferPtr(), sbus.getDataLen(), 1000);
sbus.parse();
sbusData = sbus.getData();
```

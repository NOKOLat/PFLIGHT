#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include<array>
#include<string>

template<typename _Tp, std::size_t _Nm>
struct RingBuffer: public std::array<_Tp, _Nm>{

    void push(_Tp value){
        this->data()[headIndex] = value;
        lastIndex = headIndex;
        headIndex = (lastIndex+1)%_Nm;
    }

    void setHeadIndex(uint8_t num){
        headIndex = (headIndex+num)%_Nm;
        lastIndex = (headIndex+_Nm - 1) %_Nm;
    }

    uint8_t *getBufferPtr(){
        return (uint8_t*)this->data();
    }

    _Tp& operator[](int8_t __n) noexcept{
    	return this->data()[(headIndex+__n+_Nm)%_Nm];
    }
private:
    uint8_t headIndex = 0;
    uint8_t lastIndex = _Nm-1;
};

template<typename T, std::size_t N>
std::ostream& operator<<(std::ostream& o, RingBuffer<T,N>& obj){
	for(uint8_t n=0; n<N; n++){
        o << std::to_string(obj[n]);
        if(n<N-1){
            o << std::string(", ");
        }
    }
	return o;
}

#endif /*INC_RING_BUFFER_H_*/

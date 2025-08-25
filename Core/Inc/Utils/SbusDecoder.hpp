#ifndef SBUS_DECODER_HPP
#define SBUS_DECODER_HPP

#include <cstdint>
#include <array>
#include "Sbus/sbus.h"
#include "FlightData/SbusData.hpp"

namespace nokolat {

    SbusChannelData Decode(nokolat::SBUS_DATA sbus_data){

        SbusChannelData channel_data;

        // 受信判定

        if(sbus_data[(uint8_t)SbusChannel::arm] != 0){

        	channel_data.is_receive = true;
        }
        else{

        	channel_data.is_receive = false;
        }

        //FailSafeの処理
        channel_data.failsafe_bit = sbus_data.failsafe;

        //pitch(angle)
        channel_data.target_value[0] = (float)((sbus_data[(uint8_t)SbusChannel::pitch]    - channel_data.center[(uint8_t)SbusChannel::pitch]) / (float)(channel_data.max[(uint8_t)SbusChannel::pitch]    - channel_data.center[(uint8_t)SbusChannel::pitch])) * channel_data.angle_pitch_max;

        //roll(angle)
        channel_data.target_value[1] = (float)((sbus_data[(uint8_t)SbusChannel::roll]	 - channel_data.center[(uint8_t)SbusChannel::roll])  / (float)(channel_data.max[(uint8_t)SbusChannel::roll]     - channel_data.center[(uint8_t)SbusChannel::roll]))  * channel_data.angle_roll_max;

        //yaw(rate)
        channel_data.target_value[2] = (float)((sbus_data[(uint8_t)SbusChannel::yaw]      - channel_data.center[(uint8_t)SbusChannel::yaw])   / (float)(channel_data.max[(uint8_t)SbusChannel::yaw] 	  - channel_data.center[(uint8_t)SbusChannel::yaw]))   * channel_data.rate_yaw_max;

        //throttle
        channel_data.throttle 	     = (float)((sbus_data[(uint8_t)SbusChannel::throttle] - channel_data.min[(uint8_t)SbusChannel::throttle]) / (float)(channel_data.max[(uint8_t)SbusChannel::throttle] - channel_data.min[(uint8_t)SbusChannel::throttle])) * channel_data.throttle_max;

        //armの判定
        if(sbus_data[(uint8_t)SbusChannel::arm] > 1500){

            channel_data.arm = true;
        }
        else{

            channel_data.arm = false;
        }

        //飛行開始判定
        if(sbus_data[(uint8_t)SbusChannel::fly] > 1500 && sbus_data[(uint8_t)SbusChannel::throttle] < 400){

            channel_data.fly = true;
        }
        else{

            channel_data.fly = false;
        }

        //投下判定(2段階投下を採用）
        if(sbus_data[(uint8_t)SbusChannel::drop] > 1250){

            channel_data.drop = 2;
        }
        else if(sbus_data[(uint8_t)SbusChannel::drop] > 750){

            channel_data.drop = 1;
        }
        else{

            channel_data.drop = 0;
        }

        //自動投下判定
        if(sbus_data[(uint8_t)SbusChannel::autodrop] > 1500){

            channel_data.autodrop = true;
        }
        else{

            channel_data.autodrop = false;
        }

        //自動操縦
        if(sbus_data[(uint8_t)SbusChannel::autofly]){

            channel_data.autofly = true;
        }
        else{

            channel_data.autofly = false;
        }

        return channel_data;
    }

}

#endif // SBUS_DECODER_HPP#endif // SBUS_DECODER_HPP

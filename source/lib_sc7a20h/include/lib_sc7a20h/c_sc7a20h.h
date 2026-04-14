#ifndef __ARDUINO_SENSORS_SC7A20H_H__
#define __ARDUINO_SENSORS_SC7A20H_H__
#include "rcore/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    namespace nsensors
    {
        enum ETapEvent
        {
            TAP_NONE    = 0,
            TAP_SINGLE  = 1,
            TAP_DOUBLE  = 2,
            TAP_UNKNOWN = 3,
        };
        const char* to_string(ETapEvent event);

        enum ECubeSide
        {
            CUBE_SIDE_UNKNOWN = 0,
            CUBE_SIDE_FRONT   = 1,
            CUBE_SIDE_BACK    = 2,
            CUBE_SIDE_LEFT    = 3,
            CUBE_SIDE_RIGHT   = 4,
            CUBE_SIDE_TOP     = 5,
            CUBE_SIDE_BOTTOM  = 6,
        };
        const char* to_string(ECubeSide side);

        enum EWakeUpEvent
        {
            WAKEUP_NONE     = 0,
            WAKEUP_TAP      = 1,
            WAKEUP_ROTATION = 2,
            WAKEUP_MOTION   = 4,
        };
        const char* to_string(EWakeUpEvent event);

        struct sc7a20h_t
        {
            u8  i2c_addr;
            u8  int_pin;
            u64 m_last_activity_ms;
        };

        // X, Y, Z
        void wakeup(sc7a20h_t& sensor, EWakeUpEvent& outEvent, ETapEvent& outTapEvent, ECubeSide& outSide);
        bool init(sc7a20h_t& sensor, u8 i2c_address, u8 int_pin);
        bool update(sc7a20h_t& sensor, ECubeSide& outSide, ETapEvent& outTapEvent);

    }  // namespace nsensors
}  // namespace ncore

#endif  // __ARDUINO_SENSORS_SC7A20H_H__

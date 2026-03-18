#ifndef __ARDUINO_SENSORS_RD03D_V2_H__
#define __ARDUINO_SENSORS_RD03D_V2_H__
#include "rcore/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    namespace nsensors
    {
        namespace nrd03d
        {
            struct target_t
            {
                s16 detected;  // Whether the target is detected (1) or not (0)
                s16 x;         // X coordinate in mm (left/right)
                s16 y;         // Y coordinate in mm (distance)
                s16 v;         // Speed of the target in cm/s
            };

            struct sensor_t
            {
                target_t m_target[3];
                u16      m_index;
                u8       m_buffer[32 + 6];
            };

            void begin(sensor_t& sensor, u8 rxPin, u8 txPin);
            bool update(sensor_t& sensor);
            bool getTarget(sensor_t& sensor, s8 i, target_t& t);  // get target i (0..2)

        }  // namespace nrd03d
    }  // namespace nsensors
}  // namespace ncore

#endif  // __ARDUINO_SENSORS_RD03D_V2_H__

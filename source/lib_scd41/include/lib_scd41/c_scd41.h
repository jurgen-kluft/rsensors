#ifndef __ARDUINO_SENSORS_SCD4X_H__
#define __ARDUINO_SENSORS_SCD4X_H__
#include "rcore/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    namespace nsensors
    {
        bool initSCD41();

        // outHumidity: relative humidity in percent
        // outTemperature: temperature in degrees Celsius
        // outCo2: CO2 concentration in ppm
        bool updateSCD41(f32& outHumidity, f32& outTemperature, u16& outCo2);
        
    }  // namespace nsensors
}  // namespace ncore

#endif  // __ARDUINO_SENSORS_SCD4X_H__

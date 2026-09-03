#ifndef __ARDUINO_SENSORS_HMMD_H__
#define __ARDUINO_SENSORS_HMMD_H__
#include "rcore/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    namespace nsensors
    {
        bool initHMMD(u8 rxPin, u8 txPin);
        bool readHMMD(i32* outDetection, i32* outDistanceInCm);
        bool readHMMD2(i32* outDetection, i32* outDistanceInCm);

    }  // namespace nsensors
}  // namespace ncore

#endif  // __ARDUINO_SENSORS_HMMD_H__

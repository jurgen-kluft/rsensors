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
        bool readHMMD(s8* outDetection, u16* outDistanceInCm);
        bool readHMMD2(s8* outDetection, u16* outDistanceInCm);

    }  // namespace nsensors
}  // namespace ncore

#endif  // __ARDUINO_SENSORS_HMMD_H__

#ifndef __ARDUINO_SENSORS_MG58F18_H__
#define __ARDUINO_SENSORS_MG58F18_H__
#include "rcore/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    namespace nsensors
    {
        namespace nmg58f18
        {
            bool initialize(s8 rx, s8 tx, s8 motion_detected_input_pin);  // UART pins and motion detected input pin
            bool is_detecting(s8 pin);                                    // The 'out' pin of the sensor board attached to this pin will go HIGH when motion is detected

            enum EState
            {
                STATE_OFF = 0x00,
                STATE_ON  = 0x01,
            };

            enum ETriggerMode
            {
                TRIGGER_MODE_CONTINUOUS = 0x00,
                TRIGGER_MODE_SINGLE     = 0x01,
            };

            void setSensingDistanceThreshold(u32 thresholdInMm);
            void setOutputDelayTime(u32 delayTimeInMs);
            void setLightSensorState(EState state);
            void setLockoutTime(u32 lockoutTimeInMs);
            void setEffectiveOutputLevel(u8 level);
            void setPowerConsumptionMode(u8 mode);
            void setTriggerMode(ETriggerMode mode);
            void setTransmitPowerLevel(u8 level);
            void setLightSensorThreshold(u8 threshold);
            void setPWMDutyCycleNoSensor(u32 dutyCycle);
            void setPulseWidthForPowerSupply(u8 pulseWidth);

            bool querySensingDistanceThreshold(u32& outThresholdInMm);
            bool queryOutputDelayTime(u32& outDelayTimeInMs);
            bool queryLightSensorState(EState& outState);
            bool queryLockoutTime(u32& outLockoutTimeInMs);
            bool queryEffectiveOutputLevel(u8& outLevel);
            bool queryPowerConsumptionMode(u8& outMode);
            bool querySensorTriggerStatus(u8& outStatus);
            bool queryEnvironmentBrightness(u8& outBrightness);
            bool queryFirmwareVersionNumber(u32& outVersionNumber);
            bool queryTriggerMode(ETriggerMode& outMode);
            bool queryTransmitPowerLevel(u8& outLevel);
            bool queryLightSensorThreshold(u8& outThreshold);
            bool queryPWMSwitchStatus(u8& outStatus);
            bool queryPWMDutyCycleNoSensor(u32& outDutyCycle);
            bool queryPowerSupplyPulseWidth(u8& outPulseWidth);
            bool queryCurrentSensingMode(u8& outMode);

        }  // namespace nmg58f18
    }  // namespace nsensors
}  // namespace ncore

#endif  // __ARDUINO_SENSORS_MG58F18_H__

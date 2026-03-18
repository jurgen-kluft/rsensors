#include "rsensors/c_mg58f18.h"
#include "rcore/c_gpio.h"
#include "rcore/c_timer.h"
#include "rcore/c_serial.h"

#ifdef TARGET_ARDUINO
#    include "Arduino.h"
#    include "Wire.h"
#endif

namespace ncore
{
    namespace nsensors
    {
#ifdef TARGET_ARDUINO
        namespace nmg58f18
        {
            // Documentation:

            // Serial Communication Command and Response Protocol for MG58F18
            // |---------------------------------------------------------------------------------------------
            // |  Byte 0/Head | Byte 1  | Byte 2  |   Byte 3   |   Byte 4   |    Byte 5     |  Byte 6/Tail  |
            // |       5A     |   CMD   |         |            |            |  Checksum     |      FE       |
            // |---------------------------------------------------------------------------------------------

            // Checksum (byte 5) = Byte1 ^ Byte2 ^ Byte3 ^ Byte4
            // For a QUERY, byte 2, 3 and 4 are empty (0x00).
            // For a SET, the value is placed in byte 2, 3 and 4 as big-endian (byte 2 is the MSB, byte 4 is the LSB).

            enum ECmd
            {
                CMD_SET_SENSING_DISTANCE_THRESHOLD   = 0x01,  // Set the sensing distance threshold (100-65000)
                CMD_QUERY_SENSING_DISTANCE_THRESHOLD = 0x81,  // Query the sensing distance threshold value
                CMD_SET_OUTPUT_DELAY_TIME            = 0x02,  // Set the output delay time value (0x00 - 0xFFFFFF)
                CMD_QUERY_OUTPUT_DELAY_TIME          = 0x82,  // Query output delay time value
                CMD_SET_LIGHT_SENSOR_STATE           = 0x03,  // Switching the light sensor on and off (0x00: off, 0x01: on)
                CMD_QUERY_LIGHT_SENSOR_STATE         = 0x83,  // Check the status of the light sensor switch
                CMD_SET_LOCKOUT_TIME                 = 0x04,  // Set the lockout time value (0x00 - 0xFFFFFF)
                CMD_QUERY_LOCKOUT_TIME               = 0x84,  // Query the lockout time value
                CMD_SET_EFFECTIVE_OUTPUT_LEVEL       = 0x05,  // Set the effective output level (high or low) after triggering the sensor (0x00: low, 0x01: high)
                CMD_QUERY_EFFECTIVE_OUTPUT_LEVEL     = 0x85,  // Query the valid output level parameter
                CMD_SET_POWER_CONSUMPTION_MODE       = 0x06,  // Set power consumption mode: Ultra-low power (50/60uA) or normal power (13mA) (0x00: ultra-low power, 0x01: normal power)
                CMD_QUERY_POWER_CONSUMPTION_MODE     = 0x86,  // Query power consumption mode
                CMD_QUERY_SENSOR_TRIGGER_STATUS      = 0x87,  // Query sensor trigger status
                CMD_QUERY_ENVIRONMENT_BRIGHTNESS     = 0x88,  // Query whether the current environment is bright or dark.
                CMD_QUERY_FIRMWARE_VERSION_NUMBER    = 0x89,  // Query firmware version number
                CMD_SET_TRIGGER_MODE                 = 0x0A,  // Set the trigger mode (single trigger or continuous trigger) (0x00: continuous trigger, 0x01: single trigger)
                CMD_QUERY_TRIGGER_MODE               = 0x8A,  // Query trigger mode
                CMD_SET_TRANSMIT_POWER_LEVEL         = 0x0B,  // Set the transmit power level (8 levels) (0x00 - 0x07)
                CMD_QUERY_TRANSMIT_POWER_LEVEL       = 0x8B,  // Query transmit power level
                CMD_SET_LIGHT_SENSOR_THRESHOLD       = 0x0C,  // Set the light sensor threshold value (0x00 - 0xFF)
                CMD_QUERY_LIGHT_SENSOR_THRESHOLD     = 0x8C,  // Query the light sensor threshold value
                CMD_SWITCH_PWM_FUNCTION              = 0x0D,  // Switch PWM function on and off (0x00: off, 0x01: on)
                CMD_QUERY_PWM_SWITCH_STATUS          = 0x8D,  // Query PWM switch status
                CMD_SET_PWM_DUTY_CYCLE_NO_SENSOR     = 0x0E,  // Sets the PWM duty cycle value when there is no sensor (0-3500)
                CMD_QUERY_PWM_DUTY_CYCLE_NO_SENSOR   = 0x8E,  // PWM duty cycle value when there is no sensor.
                CMD_SET_PULSE_WIDTH_FOR_POWER_SUPPLY = 0x0F,  // Pulse width for pulse power supply (0x00 - 0xFF)
                CMD_QUERY_POWER_SUPPLY_PULSE_WIDTH   = 0x8F,  // Query power supply pulse width parameter
                CMD_SELECT_SENSING_MODE              = 0x10,  // Select sensing mode: Motion sensing mode or Hand swipe sensing mode (0x00: motion sensing mode, 0x01: hand swipe sensing mode)
                CMD_QUERY_CURRENT_SENSING_MODE       = 0x11,  // Query the current sensing mode
                CMD_SAVE_SETTINGS_PARAMETERS         = 0x20   // Save the settings parameters (0x01) (Note: 100ms later you will receive the response frame)
            };

            ngpio::input_pin_t motion_sensor_pin(12);
            bool               initialize(s8 rx, s8 tx, s8 motion_detected_input_pin)
            {
                if (rx >= 0 && tx >= 0 && motion_detected_input_pin >= 0)
                {
                    motion_sensor_pin = ngpio::input_pin_t(motion_detected_input_pin);
                    nserialx::begin(nserialx::SERIAL1, nbaud::Rate9600, nconfig::MODE_8N1, rx, tx);
                    // Allow time for the Serial port to initialize
                    ntimer::delay(100);
                    return true;
                }
                return false;
            }

            // The 'out' pin of the sensor board attached to this pin will go HIGH when motion is detected
            bool is_detecting(s8 pin) { return motion_sensor_pin.is_high(); }

            // clang-format off
            inline void cmd_encoder(u8 cmd, u32 value, byte* outCommand)
            {
                outCommand[0] = 0x5A;                                                           // Frame Head
                outCommand[1] = cmd;                                                            // Command
                outCommand[2] = (value >> 16) & 0xFF;                                           // Data
                outCommand[3] = (value >> 8) & 0xFF;                                            // Data
                outCommand[4] = value & 0xFF;                                                   // Data LSB
                outCommand[5] = outCommand[1] ^ outCommand[2] ^ outCommand[3] ^ outCommand[4];  // Checksum
                outCommand[6] = 0xFE;                                                           // Frame Tail
            }

            inline bool cmd_checksum_ok(const byte* response)
            {
                const u8 checksum = response[1] ^ response[2] ^ response[3] ^ response[4];
                return (checksum == response[5]);
            }

            inline bool cmd_valid_response(const byte* response, u8 expectedCmd)
            {
                if (response[0] == 0x5A && response[1] == expectedCmd && response[6] == 0xFE)
                    return cmd_checksum_ok(response);
                return false;
            }

            inline u32 cmd_decode_value(const byte* response)
            {
                return (static_cast<u32>(response[2]) << 16) | (static_cast<u32>(response[3]) << 8) | static_cast<u32>(response[4]);
            }

            #    define CMD_ENCODE_AND_SEND(cmd, value) \
                    byte command[7];           \
                    cmd_encoder(cmd, value, command); \
                    nserialx::write(nserialx::SERIAL1, command, sizeof(command))
            // clang-format on

            void setSensingDistanceThreshold(u32 thresholdInMm) { CMD_ENCODE_AND_SEND(CMD_SET_SENSING_DISTANCE_THRESHOLD, thresholdInMm); }
            void setOutputDelayTime(u32 delayTimeInMs) { CMD_ENCODE_AND_SEND(CMD_SET_OUTPUT_DELAY_TIME, delayTimeInMs); }
            void setLightSensorState(EState state) { CMD_ENCODE_AND_SEND(CMD_SET_LIGHT_SENSOR_STATE, static_cast<u32>(state)); }
            void setLockoutTime(u32 lockoutTimeInMs) { CMD_ENCODE_AND_SEND(CMD_SET_LOCKOUT_TIME, lockoutTimeInMs); }
            void setEffectiveOutputLevel(u8 level) { CMD_ENCODE_AND_SEND(CMD_SET_EFFECTIVE_OUTPUT_LEVEL, static_cast<u32>(level)); }
            void setPowerConsumptionMode(u8 mode) { CMD_ENCODE_AND_SEND(CMD_SET_POWER_CONSUMPTION_MODE, static_cast<u32>(mode)); }
            void setTriggerMode(ETriggerMode mode) { CMD_ENCODE_AND_SEND(CMD_SET_TRIGGER_MODE, static_cast<u32>(mode)); }
            void setTransmitPowerLevel(u8 level) { CMD_ENCODE_AND_SEND(CMD_SET_TRANSMIT_POWER_LEVEL, static_cast<u32>(level)); }
            void setLightSensorThreshold(u8 threshold) { CMD_ENCODE_AND_SEND(CMD_SET_LIGHT_SENSOR_THRESHOLD, static_cast<u32>(threshold)); }
            void setPWMDutyCycleNoSensor(u32 dutyCycle) { CMD_ENCODE_AND_SEND(CMD_SET_PWM_DUTY_CYCLE_NO_SENSOR, dutyCycle); }
            void setPulseWidthForPowerSupply(u8 pulseWidth) { CMD_ENCODE_AND_SEND(CMD_SET_PULSE_WIDTH_FOR_POWER_SUPPLY, static_cast<u32>(pulseWidth)); }

            // Query Commands with obtaining response value
            bool executeQueryAndReturnResponseValue(ECmd cmd, u32& outValue)
            {
                CMD_ENCODE_AND_SEND(cmd, 0);

                const u64 startTime = ntimer::millis();
                while (nserialx::available(nserialx::SERIAL1) < 7 && (ntimer::millis() - startTime) < 100)
                {  // Wait for response
                    ntimer::delay(10);
                }

                outValue = 0xFFFFFFFF;        // Invalid value
                if (Serial.available() >= 7)  // Read response
                {
                    byte response[7];
                    Serial.readBytes(response, sizeof(response));
                    if (cmd_valid_response(response, cmd))
                    {
                        outValue = cmd_decode_value(response);
                        return true;
                    }
                }
                return false;
            }

            // All query command functions
            bool querySensingDistanceThreshold(u32& outThresholdInMm) { return executeQueryAndReturnResponseValue(CMD_QUERY_SENSING_DISTANCE_THRESHOLD, outThresholdInMm); }
            bool queryOutputDelayTime(u32& outDelayTimeInMs) { return executeQueryAndReturnResponseValue(CMD_QUERY_OUTPUT_DELAY_TIME, outDelayTimeInMs); }

            bool queryLightSensorState(EState& outState)
            {
                u32 value;
                if (executeQueryAndReturnResponseValue(CMD_QUERY_LIGHT_SENSOR_STATE, value))
                {
                    outState = value == 0 ? STATE_OFF : STATE_ON;
                    return true;
                }
                return false;
            }

            bool queryLockoutTime(u32& outLockoutTimeInMs) { return executeQueryAndReturnResponseValue(CMD_QUERY_LOCKOUT_TIME, outLockoutTimeInMs); }

            bool queryEffectiveOutputLevel(u8& outLevel)
            {
                u32 value;
                if (executeQueryAndReturnResponseValue(CMD_QUERY_EFFECTIVE_OUTPUT_LEVEL, value))
                {
                    outLevel = static_cast<u8>(value);
                    return true;
                }
                return false;
            }

            bool queryPowerConsumptionMode(u8& outMode)
            {
                u32 value;
                if (executeQueryAndReturnResponseValue(CMD_QUERY_POWER_CONSUMPTION_MODE, value))
                {
                    outMode = static_cast<u8>(value);
                    return true;
                }
                return false;
            }

            bool querySensorTriggerStatus(u8& outStatus)
            {
                u32 value;
                if (executeQueryAndReturnResponseValue(CMD_QUERY_SENSOR_TRIGGER_STATUS, value))
                {
                    outStatus = static_cast<u8>(value);
                    return true;
                }
                return false;
            }

            bool queryEnvironmentBrightness(u8& outBrightness)
            {
                u32 value;
                if (executeQueryAndReturnResponseValue(CMD_QUERY_ENVIRONMENT_BRIGHTNESS, value))
                {
                    outBrightness = static_cast<u8>(value);
                    return true;
                }
                return false;
            }

            bool queryFirmwareVersionNumber(u32& outVersionNumber) { return executeQueryAndReturnResponseValue(CMD_QUERY_FIRMWARE_VERSION_NUMBER, outVersionNumber); }

            bool queryTriggerMode(ETriggerMode& outMode)
            {
                u32 value;
                if (executeQueryAndReturnResponseValue(CMD_QUERY_TRIGGER_MODE, value))
                {
                    outMode = value == 0 ? TRIGGER_MODE_CONTINUOUS : TRIGGER_MODE_SINGLE;
                    return true;
                }
                return false;
            }

            bool queryTransmitPowerLevel(u8& outLevel)
            {
                u32 value;
                if (executeQueryAndReturnResponseValue(CMD_QUERY_TRANSMIT_POWER_LEVEL, value))
                {
                    outLevel = static_cast<u8>(value);
                    return true;
                }
                return false;
            }

            bool queryLightSensorThreshold(u8& outThreshold)
            {
                u32 value;
                if (executeQueryAndReturnResponseValue(CMD_QUERY_LIGHT_SENSOR_THRESHOLD, value))
                {
                    outThreshold = static_cast<u8>(value);
                    return true;
                }
                return false;
            }

            bool queryPWMSwitchStatus(u8& outStatus)
            {
                u32 value;
                if (executeQueryAndReturnResponseValue(CMD_QUERY_PWM_SWITCH_STATUS, value))
                {
                    outStatus = static_cast<u8>(value);
                    return true;
                }
                return false;
            }

            bool queryPWMDutyCycleNoSensor(u32& outDutyCycle) { return executeQueryAndReturnResponseValue(CMD_QUERY_PWM_DUTY_CYCLE_NO_SENSOR, outDutyCycle); }

            bool queryPowerSupplyPulseWidth(u8& outPulseWidth)
            {
                u32 value;
                if (executeQueryAndReturnResponseValue(CMD_QUERY_POWER_SUPPLY_PULSE_WIDTH, value))
                {
                    outPulseWidth = static_cast<u8>(value);
                    return true;
                }
                return false;
            }

            bool queryCurrentSensingMode(u8& outMode)
            {
                u32 value;
                if (executeQueryAndReturnResponseValue(CMD_QUERY_CURRENT_SENSING_MODE, value))
                {
                    outMode = static_cast<u8>(value);
                    return true;
                }
                return false;
            }

            bool saveSettingsParameters()
            {
                CMD_ENCODE_AND_SEND(CMD_SAVE_SETTINGS_PARAMETERS, 0x01);

                // We know that we have to wait at least 100ms here
                ntimer::delay(100);

                // Wait for response
                const u64 startTime = ntimer::millis();
                while (nserialx::available(nserialx::SERIAL1) < 7 && (ntimer::millis() - startTime) < 200)
                {
                    ntimer::delay(10);
                }

                if (Serial.available() >= 7)  // Read response
                {
                    byte response[7];
                    Serial.readBytes(response, sizeof(response));
                    return cmd_valid_response(response, CMD_SAVE_SETTINGS_PARAMETERS);
                }

                return true;
            }

        }  // namespace nmg58f18
#endif
    }  // namespace nsensors
}  // namespace ncore

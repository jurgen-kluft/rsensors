#include "lib_bh1750/c_bh1750.h"
#include "rcore/c_malloc.h"

#ifdef TARGET_ARDUINO

#    include "Arduino.h"
#    include "Wire.h"

namespace nbh1750
{
    // BH1750 I2C address
    // 0x23 is the default address for most modules
    // 0x5C is the default address for some modules
    // 0x5C is used by the Adafruit library
    // 0x23 is used by the SparkFun library
#    define BH1750_I2C_ADDRESS 0x23

// No active state
#    define BH1750_POWER_DOWN 0x00

// Waiting for measurement command
#    define BH1750_POWER_ON 0x01

// Reset data register value - not accepted in POWER_DOWN mode
#    define BH1750_RESET 0x07

// Default MTreg value
#    define BH1750_DEFAULT_MTREG 69
#    define BH1750_MTREG_MIN     31
#    define BH1750_MTREG_MAX     254

    // MIT License

    // Copyright (c) 2018 claws

    // Permission is hereby granted, free of charge, to any person obtaining a copy
    // of this software and associated documentation files (the "Software"), to deal
    // in the Software without restriction, including without limitation the rights
    // to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    // copies of the Software, and to permit persons to whom the Software is
    // furnished to do so, subject to the following conditions:

    // The above copyright notice and this permission notice shall be included in all
    // copies or substantial portions of the Software.

    // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    // IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    // FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    // AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    // LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    // OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    // SOFTWARE.

    struct BH1750
    {
        enum Mode
        {
            // Same as Power Down
            UNCONFIGURED = 0,
            // Measurement at 1 lux resolution. Measurement time is approx 120ms.
            CONTINUOUS_HIGH_RES_MODE = 0x10,
            // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
            CONTINUOUS_HIGH_RES_MODE_2 = 0x11,
            // Measurement at 4 lux resolution. Measurement time is approx 16ms.
            CONTINUOUS_LOW_RES_MODE = 0x13,
            // Measurement at 1 lux resolution. Measurement time is approx 120ms.
            ONE_TIME_HIGH_RES_MODE = 0x20,
            // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
            ONE_TIME_HIGH_RES_MODE_2 = 0x21,
            // Measurement at 4 lux resolution. Measurement time is approx 16ms.
            ONE_TIME_LOW_RES_MODE = 0x23
        };

        void  initialize(byte addr = 0x23);
        bool  begin(Mode mode = CONTINUOUS_HIGH_RES_MODE, byte addr = 0x23, TwoWire* i2c = nullptr);
        bool  configure(Mode mode);
        bool  setMTreg(byte MTreg);
        bool  measurementReady(bool maxWait = false);
        float readLightLevel();

        byte          BH1750_I2CADDR;
        byte          BH1750_MTreg = (byte)BH1750_DEFAULT_MTREG;
        Mode          BH1750_MODE  = UNCONFIGURED;
        TwoWire*      I2C;
        unsigned long lastReadTimestamp;
    };

    // Correction factor used to calculate lux. Typical value is 1.2 but can
    // range from 0.96 to 1.44. See the data sheet (p.2, Measurement Accuracy)
    // for more information.
    static const float BH1750_CONV_FACTOR = 1.2;

    /**
     * Constructor
     * @params addr Sensor address (0x76 or 0x72, see datasheet)
     *
     * On most sensor boards, it was 0x76
     */
    void BH1750::initialize(byte addr)
    {
        BH1750_I2CADDR = addr;
        I2C            = &Wire;  // Allows user to change TwoWire instance
    }

    /**
     * Configure sensor
     * @param mode Measurement mode
     * @param addr Address of the sensor
     * @param i2c TwoWire instance connected to I2C bus
     */
    bool BH1750::begin(Mode mode, byte addr, TwoWire* i2c)
    {
        // I2C is expected to be initialized outside this library
        // But, allows a different address and TwoWire instance to be used
        if (i2c)
        {
            I2C = i2c;
        }
        if (addr)
        {
            BH1750_I2CADDR = addr;
        }

        // Configure sensor in specified mode and set default MTreg
        return (configure(mode) && setMTreg(BH1750_DEFAULT_MTREG));
    }

    /**
     * Configure BH1750 with specified mode
     * @param mode Measurement mode
     */
    bool BH1750::configure(Mode mode)
    {
        // default transmission result to a value out of normal range
        byte ack = 5;

        // Check measurement mode is valid
        switch (mode)
        {
            case BH1750::CONTINUOUS_HIGH_RES_MODE:
            case BH1750::CONTINUOUS_HIGH_RES_MODE_2:
            case BH1750::CONTINUOUS_LOW_RES_MODE:
            case BH1750::ONE_TIME_HIGH_RES_MODE:
            case BH1750::ONE_TIME_HIGH_RES_MODE_2:
            case BH1750::ONE_TIME_LOW_RES_MODE:

                // Send mode to sensor
                I2C->beginTransmission(BH1750_I2CADDR);
                I2C->write((uint8_t)mode);
                ack = I2C->endTransmission();

                // Wait a few moments to wake up
                delay(10);
                break;

            default:
                // Invalid measurement mode
                Serial.println(F("[BH1750] ERROR: Invalid mode"));
                break;
        }

        // Check result code
        switch (ack)
        {
            case 0:
                BH1750_MODE       = mode;
                lastReadTimestamp = millis();
                return true;
            case 1:  // too long for transmit buffer
                Serial.println(F("[BH1750] ERROR: too long for transmit buffer"));
                break;
            case 2:  // received NACK on transmit of address
                Serial.println(F("[BH1750] ERROR: received NACK on transmit of address"));
                break;
            case 3:  // received NACK on transmit of data
                Serial.println(F("[BH1750] ERROR: received NACK on transmit of data"));
                break;
            case 4:  // other error
                Serial.println(F("[BH1750] ERROR: other error"));
                break;
            default: Serial.println(F("[BH1750] ERROR: undefined error")); break;
        }

        return false;
    }

    /**
     * Configure BH1750 MTreg value
     * MT reg = Measurement Time register
     * @param MTreg a value between 31 and 254. Default: 69
     * @return bool true if MTReg successful set
     * 		false if MTreg not changed or parameter out of range
     */
    bool BH1750::setMTreg(byte MTreg)
    {
        if (MTreg < BH1750_MTREG_MIN || MTreg > BH1750_MTREG_MAX)
        {
            Serial.println(F("[BH1750] ERROR: MTreg out of range"));
            return false;
        }
        byte ack = 5;
        // Send MTreg and the current mode to the sensor
        //   High bit: 01000_MT[7,6,5]
        //    Low bit: 011_MT[4,3,2,1,0]
        I2C->beginTransmission(BH1750_I2CADDR);
        I2C->write((0b01000 << 3) | (MTreg >> 5));
        ack = I2C->endTransmission();
        I2C->beginTransmission(BH1750_I2CADDR);
        I2C->write((0b011 << 5) | (MTreg & 0b11111));
        ack = ack | I2C->endTransmission();
        I2C->beginTransmission(BH1750_I2CADDR);
        I2C->write(BH1750_MODE);
        ack = ack | I2C->endTransmission();

        // Wait a few moments to wake up
        delay(10);

        // Check result code
        switch (ack)
        {
            case 0: BH1750_MTreg = MTreg; return true;
            case 1:  // too long for transmit buffer
                Serial.println(F("[BH1750] ERROR: too long for transmit buffer"));
                break;
            case 2:  // received NACK on transmit of address
                Serial.println(F("[BH1750] ERROR: received NACK on transmit of address"));
                break;
            case 3:  // received NACK on transmit of data
                Serial.println(F("[BH1750] ERROR: received NACK on transmit of data"));
                break;
            case 4:  // other error
                Serial.println(F("[BH1750] ERROR: other error"));
                break;
            default: Serial.println(F("[BH1750] ERROR: undefined error")); break;
        }

        return false;
    }

    /**
     * Checks whether enough time has gone to read a new value
     * @param maxWait a boolean if to wait for typical or maximum delay
     * @return a boolean if a new measurement is possible
     *
     */
    bool BH1750::measurementReady(bool maxWait)
    {
        unsigned long delaytime = 0;
        switch (BH1750_MODE)
        {
            case BH1750::CONTINUOUS_HIGH_RES_MODE:
            case BH1750::CONTINUOUS_HIGH_RES_MODE_2:
            case BH1750::ONE_TIME_HIGH_RES_MODE:
            case BH1750::ONE_TIME_HIGH_RES_MODE_2: maxWait ? delaytime = (180 * BH1750_MTreg / (byte)BH1750_DEFAULT_MTREG) : delaytime = (120 * BH1750_MTreg / (byte)BH1750_DEFAULT_MTREG); break;
            case BH1750::CONTINUOUS_LOW_RES_MODE:
            case BH1750::ONE_TIME_LOW_RES_MODE:
                // Send mode to sensor
                maxWait ? delaytime = (24 * BH1750_MTreg / (byte)BH1750_DEFAULT_MTREG) : delaytime = (16 * BH1750_MTreg / (byte)BH1750_DEFAULT_MTREG);
                break;
            default: break;
        }
        // Wait for new measurement to be possible.
        // Measurements have a maximum measurement time and a typical measurement
        // time. The maxWait argument determines which measurement wait time is
        // used when a one-time mode is being used. The typical (shorter)
        // measurement time is used by default and if maxWait is set to True then
        // the maximum measurement time will be used. See data sheet pages 2, 5
        // and 7 for more details.
        unsigned long currentTimestamp = millis();
        if (currentTimestamp - lastReadTimestamp >= delaytime)
        {
            return true;
        }
        else
            return false;
    }

    /**
     * Read light level from sensor
     * The return value range differs if the MTreg value is changed. The global
     * maximum value is noted in the square brackets.
     * @return Light level in lux (0.0 ~ 54612,5 [117758,203])
     * 	   -1 : no valid return value
     * 	   -2 : sensor not configured
     */
    float BH1750::readLightLevel()
    {
        if (BH1750_MODE == UNCONFIGURED)
        {
            Serial.println(F("[BH1750] Device is not configured!"));
            return -2.0;
        }

        // Measurement result will be stored here
        float level = -1.0;

        // Read two bytes from the sensor, which are low and high parts of the sensor
        // value
        if (2 == I2C->requestFrom((int)BH1750_I2CADDR, (int)2))
        {
            unsigned int tmp = 0;
            tmp              = I2C->read();
            tmp <<= 8;
            tmp |= I2C->read();
            level = tmp;
        }
        lastReadTimestamp = millis();

        if (level != -1.0)
        {
// Print raw value if debug enabled
#    ifdef TARGET_DEBUG
            Serial.print(F("[BH1750] Raw value: "));
            Serial.println(level);
#    endif

            if (BH1750_MTreg != BH1750_DEFAULT_MTREG)
            {
                level *= (float)((byte)BH1750_DEFAULT_MTREG / (float)BH1750_MTreg);
// Print MTreg factor if debug enabled
#    ifdef TARGET_DEBUG
                Serial.print(F("[BH1750] MTreg factor: "));
                Serial.println(String((float)((byte)BH1750_DEFAULT_MTREG / (float)BH1750_MTreg)));
#    endif
            }
            if (BH1750_MODE == BH1750::ONE_TIME_HIGH_RES_MODE_2 || BH1750_MODE == BH1750::CONTINUOUS_HIGH_RES_MODE_2)
            {
                level /= 2;
            }

            // Convert raw value to lux
            level /= BH1750_CONV_FACTOR;

// Print converted value if debug enabled
#    ifdef TARGET_DEBUG
            Serial.print(F("[BH1750] Converted float value: "));
            Serial.println(level);
#    endif
        }

        return level;
    }

}  // namespace nbh1750

namespace ncore
{
    namespace nsensors
    {
        nbh1750::BH1750* bh1750        = nullptr;
        bool             bh1750_active = false;
        u16              bh1750_lux    = 0;

        bool initBH1750(u8 i2c_address)
        {
            if (bh1750 == nullptr)
            {
                bh1750 = nsystem::construct<nbh1750::BH1750>();
                bh1750->initialize(i2c_address);
                bh1750_active = bh1750->begin();
            }
            return bh1750_active;
        }

        // Light in lux
        bool updateBH1750(u16& outLuxValue)
        {
            if (bh1750 == nullptr)
            {
                bh1750_lux  = 0;
                outLuxValue = 0;
                return false;
            }

            if (!bh1750_active)
            {
                bh1750_lux    = 0;
                outLuxValue   = 65535;
                bh1750_active = bh1750->begin();
                return bh1750_active;
            }

            if (bh1750->measurementReady())
            {
                bh1750_lux  = static_cast<s32>(bh1750->readLightLevel());
                outLuxValue = bh1750_lux;
                return true;
            }
            return false;
        }

    }  // namespace nsensors
}  // namespace ncore

#else

namespace ncore
{
    namespace nsensors
    {
        struct Bh1750Sensor
        {
            Bh1750Sensor()
                : m_Initialized(false)
                , m_Lux(42)
            {
            }
            bool m_Initialized;
            s32 m_Lux;
        };

        Bh1750Sensor bh1750;

        bool initBH1750(u8 i2c_address)
        {
            if (bh1750.m_Initialized == false)
            {
                bh1750.m_Initialized = true;
                return true;
            }
            return false;
        }

        // Light in lux
        bool updateBH1750(s32& outLuxValue)
        {
            if (bh1750.m_Initialized)
            {
                outLuxValue = bh1750.m_Lux;
                return true;
            }
            outLuxValue = -1;
            return false;
        }

    }  // namespace nsensors
}  // namespace ncore

#endif
#ifdef TARGET_ARDUINO

#    include "Arduino.h"
#    include "Wire.h"

#    include "lib_sc7a20h/c_sc7a20h.h"

namespace ncore
{
    namespace nsensors
    {
        namespace nsc7a20h
        {
#    define SC7A20_I2C_ADDR 0x18

            // Reg Table

            enum ESettings
            {
                WHO_AM_I_REG      = 0x0F,  // Device ID register, should return 0x11 for SC7A20H
                CTRL_REG1         = 0x20,  // Control register 1: Data rate, power mode, axis enable
                CTRL_REG2         = 0x21,  //
                CTRL_REG3         = 0x22,  //
                CTRL_REG4         = 0x23,  //
                ADDR_STATUS_REG   = 0x27,  //
                OUT_X_L_REG       = 0x28,  // Low byte of X-axis data
                OUT_X_H_REG       = 0x29,  // High byte of X-axis data
                OUT_Y_L_REG       = 0x2A,  // Low byte of Y-axis data
                OUT_Y_H_REG       = 0x2B,  // High byte of Y-axis data
                OUT_Z_L_REG       = 0x2C,  // Low byte of Z-axis data
                OUT_Z_H_REG       = 0x2D,  // High byte of Z-axis data
                OUT_AUTO_INC      = 0x80,  // Auto-increment for multi-byte read/write, OR with register address
                INT1_CFG_REG      = 0x30,  // Interrupt configuration register
                INT1_SRC_REG      = 0x31,  // Interrupt source register
                INT1_THS_REG      = 0x32,  // Interrupt threshold register
                INT1_DURATION_REG = 0x33,  // Interrupt duration register
                CLICK_CFG_REG     = 0x38,  // Click configuration register
                CLICK_SRC_REG     = 0x39,  // Click source register
                CLICK_THS_REG     = 0x3A,  // Click threshold register
                TIME_LIMIT        = 0x3B,  // Time limit for click detection
                TIME_LATENCY      = 0x3C,  // Time latency for click detection
                TIME_WINDOW       = 0x3D,  // Time window for click detection
                SC7A20_CHIP_ID    = 0x11,  // Chip ID for SC7A20H
            };

            bool s_i2c_write(uint8_t i2c_addr, uint8_t addr, uint8_t data)
            {
                Wire.beginTransmission(i2c_addr);
                Wire.write(addr);
                Wire.write(data);
                Wire.endTransmission();

                // TODO: Check return values from wire calls.
                return true;
            }

            bool s_i2c_read(uint8_t i2c_addr, uint8_t addr, uint8_t* data, uint8_t length)
            {
                uint8_t nBytesRead(0);

                Wire.beginTransmission(i2c_addr);
                Wire.write(addr);
                Wire.endTransmission();

                Wire.requestFrom(i2c_addr, length);
                while (Wire.available())
                {
                    data[nBytesRead++] = Wire.read();
                }

                return nBytesRead == length;
            }

            void configure(sc7a20h_t& sensor)
            {
                // If the ESP32 is waking up from deepsleep, the SC7A20H may still be configured and active.
                // In that case, skip initialization of the sensor to avoid resetting the tap/wake-up detection
                // state machine.

                // Enable accelerometer: 100 Hz, all axes on
                s_i2c_write(sensor.i2c_addr, CTRL_REG1, 0x57);  // ODR=100Hz, XYZ enable

                // ±2g range (best resolution for tilt)
                s_i2c_write(sensor.i2c_addr, CTRL_REG4, 0x00);

                // -------------------------------------------------
                // Accelerometer base config
                // -------------------------------------------------

                // 25 Hz, low-power, XYZ enabled
                s_i2c_write(sensor.i2c_addr, CTRL_REG1, 0x37);

                // ±2g scale
                s_i2c_write(sensor.i2c_addr, CTRL_REG4, 0x00);

                // High-pass filter for wake-up (ignore gravity drift)
                s_i2c_write(sensor.i2c_addr, CTRL_REG2, 0x01);

                // -------------------------------------------------
                // 1. Motion / wake-up detection
                // -------------------------------------------------

                // Wake-up threshold (~0.2g)
                s_i2c_write(sensor.i2c_addr, INT1_THS_REG, 0x03);

                // Must be still for ~10 seconds to be "asleep"
                s_i2c_write(sensor.i2c_addr, INT1_DURATION_REG, 0x0A);

                // Enable wake on X / Y / Z high events
                // s_i2c_write(sensor.i2c_addr, INT1_CFG_REG, 0b00101010);

                // -------------------------------------------------
                // 2. Rotation detection (6D orientation)
                // -------------------------------------------------

                // Enable 6D on INT1
                s_i2c_write(sensor.i2c_addr, INT1_CFG_REG, 0b00101010 | 0b01000000);

                // -------------------------------------------------
                // 3. Tap detection (single + double)
                // -------------------------------------------------

                s_i2c_write(sensor.i2c_addr, CLICK_CFG_REG, 0b00111111);  // ZS | ZD | YS | YD | XS | XD
                s_i2c_write(sensor.i2c_addr, CLICK_THS_REG, 0x12);        // Tap threshold (~0.6g)
                s_i2c_write(sensor.i2c_addr, TIME_LIMIT, 0x10);           // Max tap duration (in ODR steps)
                s_i2c_write(sensor.i2c_addr, TIME_LATENCY, 0x20);         // Quiet time between taps
                s_i2c_write(sensor.i2c_addr, TIME_WINDOW, 0x40);          // Max time allowed for second tap

                // -------------------------------------------------
                // 4. Route ALL events to INT1
                // -------------------------------------------------

                // INT1 gets wake-up + 6D + click
                s_i2c_write(sensor.i2c_addr, CTRL_REG3, 0b11000000);

                ntimer::delay(100);  // Delay to allow sensor to stabilize after configuration
            }

            static volatile bool g_interrupt_pending = false;
            void                 handle_interrupt() { g_interrupt_pending = true; }

            bool init(sc7a20h_t& sensor, u8 i2c_address, u8 int_pin)
            {
                sensor.i2c_addr = i2c_address;
                sensor.int_pin  = int_pin;

                input_pin_t intPin(sensor.int_pin);
                intPin.setup();
                intPin.interruptOnRising(handle_interrupt);

                if (nwakeup::reason() != nwakeup::REASON_UNDEFINED)
                {
                    // Cold boot or reset, configure the sensor from scratch
                    configure(sensor);
                }
                sensor.m_last_activity_ms = ntimer::millis();
                return true;
            }

            bool read_cube_side(sc7a20h_t& sensor, ECubeSide& outSide)
            {
                outSide = CUBE_SIDE_UNKNOWN;

                uint8_t data[6];
                if (!s_i2c_read(sensor.i2c_addr, OUT_X_L_REG | OUT_AUTO_INC, data, sizeof(data)))
                {
                    return false;
                }

                const int16_t x = (int16_t)(raw[1] << 8 | raw[0]);
                const int16_t y = (int16_t)(raw[3] << 8 | raw[2]);
                const int16_t z = (int16_t)(raw[5] << 8 | raw[4]);

                const int16_t ax = abs(x);
                const int16_t ay = abs(y);
                const int16_t az = abs(z);

                if (az >= ax && az >= ay)
                {
                    outSide = (z >= 0) ? CUBE_SIDE_TOP : CUBE_SIDE_BOTTOM;
                }
                else if (ax >= ay)
                {
                    outSide = (x >= 0) ? CUBE_SIDE_RIGHT : CUBE_SIDE_LEFT;
                }
                else
                {
                    outSide = (y >= 0) ? CUBE_SIDE_FRONT : CUBE_SIDE_BACK;
                }
                return outSide;
            }

            void wakeup(sc7a20h_t& sensor, EWakeUpEvent& outEvent, ETapEvent& outTapEvent, ECubeSide& outSide)
            {
                uint8_t int1src, clicksrc;
                int1src  = s_i2c_read(sensor.i2c_addr, INT1_SRC_REG);
                clicksrc = s_i2c_read(sensor.i2c_addr, CLICK_SRC_REG);

                outTapEvent = TAP_NONE;
                if (clicksrc & 0x20)
                {
                    outEvent |= WAKEUP_TAP;
                    outTapEvent = TAP_DOUBLE;
                }
                else if (clicksrc & 0x10)
                {
                    outEvent |= WAKEUP_TAP;
                    outTapEvent = TAP_SINGLE;
                }

                if (int1src & 0x40)
                    outEvent |= WAKEUP_ROTATION;
                else if (int1src & 0x2A)
                    outEvent |= WAKEUP_MOTION;

                read_cube_side(sensor, outSide);
            }

            static inline int16_t convertToInt16(uint8_t lowByte, uint8_t highByte)
            {
                const uint16_t rawValue = (static_cast<uint16_t>(highByte) << 8) | static_cast<uint16_t>(lowByte);
                return static_cast<int16_t>(rawValue) >> 4;
            }

            bool read_tap_event(sc7a20h_t& sensor, ETapEvent& outTapEvent)
            {
                uint8_t clicksrc;
                s_i2c_read(sensor.i2c_addr, CLICK_SRC_REG, &clicksrc, 1);

                if (clicksrc & 0x20)
                    outTapEvent = TAP_DOUBLE;
                else if (clicksrc & 0x10)
                    outTapEvent = TAP_SINGLE;
                else
                    outTapEvent = TAP_NONE;

                return outTapEvent != TAP_NONE;
            }

            void update(sc7a20h_t& sensor, ECubeSide& outSide, ETapEvent& outTapEvent, bool& outActive)
            {
                const u64 inactivity_threshold_ms = 1000;  // 1 second
                const u64 now                     = ntimer::millis();

                read_tap_event(sensor, outTapEvent);
                read_cube_side(sensor, outSide);

                if (g_interrupt_pending)
                {
                    g_interrupt_pending       = false;
                    sensor.m_last_activity_ms = now;
                    outSide                   = CUBE_SIDE_UNKNOWN;
                    outActive                 = true;
                }
                else
                {
                    if (now > (sensor.m_last_activity_ms + inactivity_threshold_ms))
                    {
                        outActive = false;
                    }
                }
            }
        }  // namespace nsc7a20h

        void wakeup(sc7a20h_t& sensor, EWakeUpEvent& outEvent, ETapEvent& outTapEvent, ECubeSide& outSide) { nsc7a20h::wakeup(sensor, outEvent, outTapEvent, outSide); }
        bool init(sc7a20h_t& sensor, u8 i2c_address, u8 int_pin) { return nsc7a20h::init(sensor, i2c_address, int_pin); }
        bool update(sc7a20h_t& sensor, ECubeSide& outSide, ETapEvent& outTapEvent) { return nsc7a20h::update(sensor, outSide, outTapEvent); }
    }  // namespace nsensors
}  // namespace ncore

#else

#    include "lib_sc7a20h/c_sc7a20h.h"

namespace ncore
{
    namespace nsensors
    {
        void wakeup(sc7a20h_t& sensor, EWakeUpEvent& outEvent, ETapEvent& outTapEvent, ECubeSide& outSide)
        {
            outEvent    = WAKEUP_NONE;
            outTapEvent = TAP_NONE;
            outSide     = CUBE_SIDE_UNKNOWN;
        }

        bool init(sc7a20h_t& sensor, u8 i2c_address)
        {
            sensor.i2c_addr           = i2c_address;
            sensor.m_last_activity_ms = 0;
            return true;
        }

        bool update(sc7a20h_t& sensor, ECubeSide& outSide, ETapEvent& outTapEvent)
        {
            outSide     = CUBE_SIDE_UNKNOWN;
            outTapEvent = TAP_UNKNOWN;
            return false;
        }

    }  // namespace nsensors
}  // namespace ncore

#endif

namespace ncore
{
    namespace nsensors
    {
        const char* to_string(ETapEvent event)
        {
            switch (event)
            {
                case TAP_NONE: return "None";
                case TAP_SINGLE: return "Single Tap";
                case TAP_DOUBLE: return "Double Tap";
                default: return "Unknown";
            }
        }

        const char* to_string(ECubeSide side)
        {
            switch (side)
            {
                case CUBE_SIDE_FRONT: return "Front";
                case CUBE_SIDE_BACK: return "Back";
                case CUBE_SIDE_LEFT: return "Left";
                case CUBE_SIDE_RIGHT: return "Right";
                case CUBE_SIDE_TOP: return "Top";
                case CUBE_SIDE_BOTTOM: return "Bottom";
                default: return "Unknown";
            }
        }

        const char* to_string(EWakeUpEvent event)
        {
            switch (event)
            {
                case WAKEUP_NONE: return "None";
                case WAKEUP_SINGLE_TAP: return "Single Tap";
                case WAKEUP_DOUBLE_TAP: return "Double Tap";
                case WAKEUP_ROTATION: return "Rotation";
                case WAKEUP_MOTION: return "Motion";
                default: return "Unknown";
            }
        }
    }  // namespace nsensors
}  // namespace ncore

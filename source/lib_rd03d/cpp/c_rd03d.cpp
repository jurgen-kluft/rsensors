#include "rcore/c_log.h"
#include "rcore/c_timer.h"
#include "rcore/c_serial.h"
#include "lib_rd03d/c_rd03d.h"

#ifdef TARGET_ARDUINO
#    include "Arduino.h"
#    include "Wire.h"
#endif

#ifdef TARGET_ARDUINO

namespace ncore
{
    namespace nsensors
    {
        namespace nrd03d
        {
            enum EState
            {
                WAIT_AA,
                WAIT_FF,
                WAIT_03,
                WAIT_00,
                RECEIVE_FRAME
            };

            bool parseData(sensor_t& sensor, const u8* buffer, u32 len);

            void begin(sensor_t& sensor, u8 rxPin, u8 txPin)
            {
                nbaud::Enum baud = nbaud::Rate256000;
                nserialx::begin(nserialx::SERIAL1, baud, nconfig::MODE_8N1, rxPin, txPin);

                sensor.m_index = 0;
                for (s8 i = 0; i < 3; i++)
                {
                    sensor.m_target[i].x        = 0;
                    sensor.m_target[i].y        = 0;
                    sensor.m_target[i].v        = 0;
                    sensor.m_target[i].detected = 0;
                }
            }

            // Parser state-machine for UART data
            bool update(sensor_t& sensor)
            {
                bool   data_updated = false;
                EState state        = WAIT_AA;

                sensor.m_index = 0;

                while (nserialx::available(nserialx::SERIAL1))
                {
                    byte byteIn;
                    nserialx::read_bytes(nserialx::SERIAL1, &byteIn, 1);

                    switch (state)
                    {
                        case WAIT_AA:
                            if (byteIn == 0xAA)
                                state = WAIT_FF;
                            break;

                        case WAIT_FF:
                            if (byteIn == 0xFF)
                                state = WAIT_03;
                            else
                                state = WAIT_AA;
                            break;

                        case WAIT_03:
                            if (byteIn == 0x03)
                                state = WAIT_00;
                            else
                                state = WAIT_AA;
                            break;

                        case WAIT_00:
                            if (byteIn == 0x00)
                            {
                                sensor.m_index = 0;
                                state          = RECEIVE_FRAME;
                            }
                            else
                                state = WAIT_AA;
                            break;

                        case RECEIVE_FRAME:
                            sensor.m_buffer[sensor.m_index++] = byteIn;
                            if (sensor.m_index >= 26)
                            {  // 24 bytes data + 2 tail bytes
                                if (sensor.m_buffer[24] == 0x55 && sensor.m_buffer[25] == 0xCC)
                                {
                                    data_updated = parseData(sensor, sensor.m_buffer, 24);
                                }
                                state          = WAIT_AA;
                                sensor.m_index = 0;
                            }
                            break;
                    }
                }
                return data_updated;
            }

            bool parseData(sensor_t& sensor, const u8* buf, u32 len)
            {
                if (len != 24)
                    return false;

                for (s16 i = 0; i < 3; i++, buf += 8)
                {
                    const uint16_t raw_x = ((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));  // x coordinate (left/right)
                    const uint16_t raw_y = ((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));  // y coordinate (depth/distance)
                    const uint16_t raw_v = ((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));  // v speed
                    const uint16_t raw_s = ((uint16_t)buf[6] | ((uint16_t)buf[7] << 8));  // s distance unit (fixed value, 360 mm)

                    if (!(raw_x == 0 && raw_y == 0 && raw_v == 0 && raw_s == 0))
                    {
                        target_t& t = sensor.m_target[i];
                        t.detected  = 1;

                        t.x = (raw_x & 0x8000) ? (raw_x & 0x7FFF) : -1 * (raw_x & 0x7FFF);  // mm
                        t.y = (raw_y & 0x8000) ? (raw_y & 0x7FFF) : -1 * (raw_y & 0x7FFF);  // mm
                        t.v = (raw_v & 0x8000) ? (raw_v & 0x7FFF) : -1 * (raw_v & 0x7FFF);  // cm/s

                        t.x = t.x / 10;  // convert to cm
                        t.y = t.y / 10;  // convert to cm
                    }
                    else
                    {
                        target_t& t = sensor.m_target[i];
                        t.detected  = 0;
                        t.x         = 0;
                        t.y         = 0;
                        t.v         = 0;
                    }
                }

                return true;
            }

            bool getTarget(sensor_t& sensor, s8 i, target_t& t)
            {
                if (i < 0 || i >= 3)
                    return false;
                t = sensor.m_target[i];
                return t.detected == 1;
            }

        }  // namespace nrd03d
    }  // namespace nsensors
}  // namespace ncore

#else
namespace ncore
{
    namespace nsensors
    {
        namespace nrd03d
        {
            void begin(sensor_t& sensor, u8 rxPin, u8 txPin) {}
            bool update(sensor_t& sensor) { return false; }
            bool getTarget(sensor_t& sensor, s8 i, target_t& t)
            {
                t.x = 0;
                t.y = 0;
                t.v = 0;
                return false;
            }

        }  // namespace nrd03d
    }  // namespace nsensors
}  // namespace ncore

#endif

#include "rdno_core/c_log.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_malloc.h"
#include "rdno_core/c_serial.h"
#include "rdno_sensors/c_hsp24.h"
#include "rdno_sensors/c_frame_reader.h"

#include "ccore/c_memory.h"
#include "ccore/c_printf.h"
#include "ccore/c_stream.h"

#ifdef TARGET_ARDUINO
#    include "Arduino.h"
#    include "Wire.h"
#endif

namespace ncore
{
    namespace nsensors
    {
        namespace nseeed
        {
#define FRAME_START_SIZE               4
#define FRAME_END_SIZE                 4
#define FRAME_INTRAFRAME_LENGTH_OFFSET 4
#define FRAME_INTRAFRAME_CMD_OFFSET    6
#define FRAME_INTRAFRAME_DATA_OFFSET   6

            static const byte SEND_AND_ACK_FRAME_START[FRAME_START_SIZE] = {0xFD, 0xFC, 0xFB, 0xFA};
            static const byte SEND_AND_ACK_FRAME_END[FRAME_END_SIZE]     = {0x04, 0x03, 0x02, 0x01};

            static const byte REPORT_FRAME_START[FRAME_START_SIZE] = {0xF4, 0xF3, 0xF2, 0xF1};
            static const byte REPORT_FRAME_END[FRAME_END_SIZE]     = {0xF8, 0xF7, 0xF6, 0xF5};

            static const nserial::frame_sequence_t SEND_AND_ACK_FRAME_START_SEQUENCE(SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
            static const nserial::frame_sequence_t SEND_AND_ACK_FRAME_END_SEQUENCE(SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));

            static const nserial::frame_sequence_t REPORT_FRAME_START_SEQUENCE(REPORT_FRAME_START, DARRAYSIZE(REPORT_FRAME_START));
            static const nserial::frame_sequence_t REPORT_FRAME_END_SEQUENCE(REPORT_FRAME_END, DARRAYSIZE(REPORT_FRAME_END));

            static nserial::frame_sequence_t const *FRAME_START_SEQUENCES[] = {&REPORT_FRAME_START_SEQUENCE, &SEND_AND_ACK_FRAME_START_SEQUENCE};
            static nserial::frame_sequence_t const *FRAME_END_SEQUENCES[]   = {&REPORT_FRAME_END_SEQUENCE, &SEND_AND_ACK_FRAME_END_SEQUENCE};

#define SERIAL_RCV_BUFFER_SIZE 128
#define SERIAL_RCV_TIMEOUT     100

            struct hsp24_t
            {
                reader_t               *mStreamReader;
                writer_t               *mStreamWriter;
                byte                    mSerialReceiveBuffer[SERIAL_RCV_BUFFER_SIZE];
                nserial::frame_data_t  *mFrameData;
                nserial::frame_reader_t mFrameReader;
            };

            hsp24_t *create_hsp24(ncore::reader_t *serial)
            {
                hsp24_t *sensor = (hsp24_t *)ncore::nsystem::malloc(sizeof(hsp24_t));
                if (sensor)
                {
                    sensor->mFrameReader.initialize(serial, sensor->mSerialReceiveBuffer, SERIAL_RCV_BUFFER_SIZE);
                    sensor->mFrameData    = (nserial::frame_data_t *)ncore::nsystem::malloc(2 * sizeof(nserial::frame_data_t));
                    sensor->mFrameData[0] = nserial::frame_data_t(4 * 2 + 2, 64);  // Start min/max frame length
                    sensor->mFrameData[1] = nserial::frame_data_t(4 * 2 + 2, 64);  // Start min/max frame length
                    sensor->mFrameReader.set_frame_data(FRAME_START_SEQUENCES, FRAME_END_SEQUENCES, sensor->mFrameData, 2);
                }
                return sensor;
            }

            enum ECommand
            {
                ENABLE_CONFIGURATION          = 0x00FF,  // Enable configuration commands
                END_CONFIGURATION             = 0x00FE,  // End configuration command
                SET_MAX_DISTANCE_UNOCCUPIED   = 0x0060,  // Configure max distance gate & unoccupied duration
                READ_PARAMETERS               = 0x0061,  // Read radar configuration parameters
                ENABLE_ENGINEERING_MODE       = 0x0062,  // Enable engineering mode
                CLOSE_ENGINEERING_MODE        = 0x0063,  // Close engineering mode
                SET_DISTANCE_GATE_SENSITIVITY = 0x0064,  // Configure distance gate sensitivity
                READ_FIRMWARE_VERSION         = 0x00A0,  // Read firmware version
                SET_SERIAL_BAUD_RATE          = 0x00A1,  // Set serial port baud rate
                RESTORE_FACTORY_SETTINGS      = 0x00A2,  // Restore factory settings
                RESTART_MODULE                = 0x00A3,  // Restart module
                BLUETOOTH_SETTINGS            = 0x00A4,  // Bluetooth on/off
                GET_MAC_ADDRESS               = 0x00A5,  // Query MAC address
                OBTAIN_BLUETOOTH_PERMISSIONS  = 0x00A8,  // Obtain Bluetooth permissions
                SET_BLUETOOTH_PASSWORD        = 0x00A9,  // Set Bluetooth password
                SET_DISTANCE_RESOLUTION       = 0x00AA,  // Configure distance resolution
                QUERY_DISTANCE_RESOLUTION     = 0x00AB   // Query distance resolution
            };

            // ---------------------------------------------------------------------------------------------------------------------------------
            // Command encoding functions
            // ---------------------------------------------------------------------------------------------------------------------------------

            inline void writeSequence(u8 *&outBuffer, u8 const *inSequence, s32 length)
            {
                for (s32 i = 0; i < length; i++)
                    *outBuffer++ = inSequence[i];
            }

            inline void skipIntraFrameLength(u8 *&outBuffer)
            {
                outBuffer += 2;  // Reserve 2 bytes for length
            }

            inline void writePayloadLength(u8 *lengthPos, u16 length)
            {
                lengthPos[0] = (length >> 8) & 0xFF;
                lengthPos[1] = length & 0xFF;
            }

            inline void writeCmd(u8 *&outBuffer, u16 cmd)
            {
                *outBuffer++ = (cmd >> 8) & 0xFF;
                *outBuffer++ = cmd & 0xFF;
            }

            inline void writeUInt16LE(u8 *&outBuffer, u16 value)
            {
                *outBuffer++ = value & 0xFF;
                *outBuffer++ = (value >> 8) & 0xFF;
            }

            inline void writeUInt32LE(u8 *&outBuffer, u32 value)
            {
                *outBuffer++ = value & 0xFF;
                *outBuffer++ = (value >> 8) & 0xFF;
                *outBuffer++ = (value >> 16) & 0xFF;
                *outBuffer++ = (value >> 24) & 0xFF;
            }

            s32 encodeEnableConfiguration(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, ENABLE_CONFIGURATION);
                writeUInt16LE(outBuffer, 0x0001);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            s32 encodeDisableConfiguration(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, END_CONFIGURATION);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            s32 encodeSetMaxDistanceUnoccupied(u8 *outBuffer, u16 motionGateWord, u32 motionGateVal, u16 restGateWord, u32 restGateVal, u16 unoccupiedWord, u32 unoccupiedVal)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, SET_MAX_DISTANCE_UNOCCUPIED);
                // Payload, 3 * (word + dword) = 18 bytes
                writeUInt16LE(outBuffer, motionGateWord);
                writeUInt32LE(outBuffer, motionGateVal);
                writeUInt16LE(outBuffer, restGateWord);
                writeUInt32LE(outBuffer, restGateVal);
                writeUInt16LE(outBuffer, unoccupiedWord);
                writeUInt32LE(outBuffer, unoccupiedVal);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Read Parameters
            s32 encodeReadParameters(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, READ_PARAMETERS);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Enable Engineering Mode
            s32 encodeEnableEngineeringMode(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, ENABLE_ENGINEERING_MODE);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Close Engineering Mode
            s32 encodeCloseEngineeringMode(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, CLOSE_ENGINEERING_MODE);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Set Distance Gate Sensitivity
            s32 encodeSetDistanceGateSensitivity(u8 *outBuffer, u16 gateWord, u32 gateVal, u16 motionWord, u32 motionVal, u16 staticWord, u32 staticVal)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, SET_DISTANCE_GATE_SENSITIVITY);
                writeUInt16LE(outBuffer, gateWord);
                writeUInt32LE(outBuffer, gateVal);
                writeUInt16LE(outBuffer, motionWord);
                writeUInt32LE(outBuffer, motionVal);
                writeUInt16LE(outBuffer, staticWord);
                writeUInt32LE(outBuffer, staticVal);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Read Firmware Version
            s32 encodeReadFirmwareVersion(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, READ_FIRMWARE_VERSION);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Set Serial Baud Rate
            s32 encodeSetSerialBaudRate(u8 *outBuffer, u16 baudIndex)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, SET_SERIAL_BAUD_RATE);
                writeUInt16LE(outBuffer, baudIndex);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Restore Factory Settings
            s32 encodeRestoreFactorySettings(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, RESTORE_FACTORY_SETTINGS);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Restart Module
            s32 encodeRestartModule(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, RESTART_MODULE);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Bluetooth Settings
            s32 encodeBluetoothSettings(u8 *outBuffer, bool enable)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, BLUETOOTH_SETTINGS);
                writeUInt16LE(outBuffer, enable ? 0x0100 : 0x0000);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Get MAC Address
            s32 encodeGetMacAddress(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, GET_MAC_ADDRESS);
                writeUInt16LE(outBuffer, 0x0001);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Obtain Bluetooth Permissions
            s32 encodeObtainBluetoothPermissions(u8 *outBuffer, const char *password)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, OBTAIN_BLUETOOTH_PERMISSIONS);
                for (s32 i = 0; i < 6; i++)
                    writeUInt16LE(outBuffer, (u16)password[i]);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Set Bluetooth Password
            s32 encodeSetBluetoothPassword(u8 *outBuffer, const char *password)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, SET_BLUETOOTH_PASSWORD);
                for (s32 i = 0; i < 6; i++)
                    writeUInt16LE(outBuffer, (u16)password[i]);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Set Distance Resolution
            s32 encodeSetDistanceResolution(u8 *outBuffer, u16 resolutionIndex)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, SET_DISTANCE_RESOLUTION);
                writeUInt16LE(outBuffer, resolutionIndex);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // Query Distance Resolution
            s32 encodeQueryDistanceResolution(u8 *outBuffer)
            {
                u8 const *const outStart = outBuffer;
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_START, DARRAYSIZE(SEND_AND_ACK_FRAME_START));
                skipIntraFrameLength(outBuffer);
                writeCmd(outBuffer, QUERY_DISTANCE_RESOLUTION);
                writePayloadLength((u8 *)(outStart + FRAME_INTRAFRAME_LENGTH_OFFSET), (u16)(outBuffer - (outStart + FRAME_INTRAFRAME_DATA_OFFSET)));
                writeSequence(outBuffer, SEND_AND_ACK_FRAME_END, DARRAYSIZE(SEND_AND_ACK_FRAME_END));
                return (outBuffer - outStart);
            }

            // ---------------------------------------------------------------------------------------------------------------------------------
            // ACK functions
            // ---------------------------------------------------------------------------------------------------------------------------------
            // Universal ACK validator
            inline bool validateAck(u16 expectedCmd, const u8 *buffer, size_t length)
            {
                if (length < 12)
                    return false;  // Minimum ACK size
                if (!(buffer[0] == 0xFD && buffer[1] == 0xFC && buffer[2] == 0xFB && buffer[3] == 0xFA))
                    return false;
                const u16 cmd = (buffer[6] << 8) | buffer[7];
                if (cmd != (expectedCmd | 0x0100))
                    return false;
                const u16 ackStatus = (buffer[8] << 8) | buffer[9];
                return ackStatus == 0x0001;  // Success
            }

            // Utility readers
            inline u16 readUInt16LE(const u8 *buffer) { return (u16)buffer[0] | ((u16)buffer[1] << 8); }
            inline u32 readUInt32LE(const u8 *buffer) { return (u32)buffer[0] | ((u32)buffer[1] << 8) | ((u32)buffer[2] << 16) | ((u32)buffer[3] << 24); }

            // // Struct for ReadParameters
            // struct RadarParameters
            // {
            //     u8  maxDistanceGate;
            //     u8  motionGate;
            //     u8  restGate;
            //     u8  motionSensitivity[9];
            //     u8  stationarySensitivity[9];
            //     u16 unoccupiedDuration;
            // };

            // Parse functions
            inline void parseEnableConfigurationAck(const u8 *buffer, u16 &protocolVersion, u16 &bufferSize)
            {
                protocolVersion = readUInt16LE(&buffer[10]);
                bufferSize      = readUInt16LE(&buffer[12]);
            }

            inline void parseReadParametersAck(const u8 *buffer, RadarParameters &params)
            {
                size_t idx = 10;  // After ACK status
                idx++;            // Skip header 0xAA
                params.maxDistanceGate = buffer[idx++];
                params.motionGate      = buffer[idx++];
                params.restGate        = buffer[idx++];
                for (int i = 0; i < 9; i++)
                    params.motionSensitivity[i] = buffer[idx++];
                for (int i = 0; i < 9; i++)
                    params.stationarySensitivity[i] = buffer[idx++];
                params.unoccupiedDuration = readUInt16LE(&buffer[idx]);
            }

            inline void parseReadFirmwareVersionAck(const u8 *buffer, u16 &firmwareType, u16 &majorVersion, u32 &minorVersion)
            {
                firmwareType = readUInt16LE(&buffer[10]);
                majorVersion = readUInt16LE(&buffer[12]);
                minorVersion = readUInt32LE(&buffer[14]);
            }

            inline void parseGetMacAddressAck(const u8 *buffer, u8 mac[6])
            {
                for (int i = 0; i < 6; i++)
                    mac[i] = buffer[10 + i];
            }

            inline void parseQueryDistanceResolutionAck(const u8 *buffer, u16 &resolutionIndex) { resolutionIndex = readUInt16LE(&buffer[10]); }

            EResult parseRadarStatusAck(const u8 *msg, s32 msgLen, RadarStatus &status)
            {
                if (msg[7] != 0xAA)
                {
                    status.targetStatus = TargetStatusFrameError;
                    return Fail;
                }

                status.radarMode    = (ERadarMode)msg[6];
                status.targetStatus = (ETargetStatus)msg[8];

                if (status.targetStatus != TargetStatusFrameError)
                {
                    status.detectionDistance = (s32)readUInt16LE(&msg[15]);
                }
                else
                {
                    status.detectionDistance = -1;
                }

                if (status.radarMode == RadarMode_Engineering)
                {
                    // Parse engineering mode data
                    s32 offset                         = 17;
                    status.maximumMovementDistanceDoor = (s32)msg[offset++];
                    status.maximumRestingDistanceDoor  = (s32)msg[offset++];
                    for (s32 i = 0; i < 9; i++)
                    {
                        status.movementDistanceGateEnergy.gate[i] = (s32)msg[offset++];
                    }
                    for (s32 i = 0; i < 9; i++)
                    {
                        status.stationaryDistanceGateEnergy.gate[i] = (s32)msg[offset++];
                    }
                    status.photosensitive = (s32)msg[offset];
                }
                else
                {
                    // Basic mode, set defaults
                    status.maximumMovementDistanceDoor = 0xFF;
                    status.maximumRestingDistanceDoor  = 0xFF;
                    status.movementDistanceGateEnergy.reset();
                    status.stationaryDistanceGateEnergy.reset();
                    status.photosensitive = 0xFF;
                }
                return Success;
            }

            // ---------------------------------------------------------------------------------------------------------------------------------
            // ---------------------------------------------------------------------------------------------------------------------------------

            void MovementEnergy::reset()
            {
                for (s32 i = 0; i < 9; i++)
                    gate[i] = 0xFF;
            }
            void StationaryEnergy::reset()
            {
                for (s32 i = 0; i < 9; i++)
                    gate[i] = 0xFF;
            }

            void FirmwareVersion::toString(char *outStr, s32 outStrSize)
            {
                // Output format: "1.07.22091516"
                ncore::snprintf(outStr, outStrSize, "%u.%2u.%x", va_t(major >> 8), va_t(major & 0xFF), va_t(minor));
            }

            void initRadarStatus(RadarStatus &status)
            {
                status.targetStatus = TargetStatusFrameError;
                status.radarMode    = RadarMode_Normal;

                status.movementTargetDistance   = 0xFFFF;
                status.movementTargetEnergy     = 0xFF;
                status.stationaryTargetDistance = 0xFFFF;
                status.stationaryTargetEnergy   = 0xFF;
                status.detectionDistance        = 0xFFFF;

                // u8               maximumMovementDistanceDoor;
                // u8               maximumRestingDistanceDoor;
                // MovementEnergy   movementDistanceGateEnergy;    // Movement energy value for each distance gate
                // StationaryEnergy stationaryDistanceGateEnergy;  // Stationary energy value for each distance gate
                // u8               photosensitive;                // Photosensitive value 0-255
                status.maximumMovementDistanceDoor = 0xFF;
                status.maximumRestingDistanceDoor  = 0xFF;
                status.movementDistanceGateEnergy.reset();
                status.stationaryDistanceGateEnergy.reset();
                status.photosensitive = 0xFF;
            }

            void begin(hsp24_t *sensor, reader_t *serial_reader, writer_t *serial_writer)
            {
                sensor->mStreamReader = serial_reader;
                sensor->mStreamWriter = serial_writer;
            }

            EResult getStatus(hsp24_t *sensor, RadarStatus &status)
            {
                // Example:
                // F4F3F2F1   Start
                // 0D00       Length of data frame
                // 02         Type: 02 - Normal report frame, 01 - Engineering report frame
                // AA         Header
                // 03         Target status: 00 - No target, 01 - Moving target, 02 - Static target, 03 - Both, 04 - Frame error
                // 4400       Movement target distance (cm)
                // 3E         Movement target energy
                // 3800       Stationary target distance (cm)
                // 64         Stationary target energy
                // 3000       Detection distance (cm)
                // 55         Tail
                // 00         Calibration
                // F8F7F6F5   End

                nserial::frame_result_t result;
                if (sensor->mFrameReader.read(result))
                {
                    if (result.sequenceIndex == 0)  // Report frame?
                    {
                        return parseRadarStatusAck(result.frameStart, result.frameLength, status);
                    }
                }

                return Success;
            }

            EResult waitForAck(hsp24_t *sensor, u16 cmd, nserial::frame_result_t &frame_result)
            {
                u64 startTime = ncore::ntimer::millis();
                while ((ncore::ntimer::millis() - startTime) < SERIAL_RCV_TIMEOUT)
                {
                    if (sensor->mFrameReader.read(frame_result))
                    {
                        if (frame_result.sequenceIndex == 1)  // ACK frame
                        {
                            if (validateAck(cmd, frame_result.frameStart, frame_result.frameLength))
                            {
                                return Success;
                            }
                            else
                            {
                                return Fail;
                            }
                        }
                    }
                }
                return Timeout;
            }

            EResult enableConfigMode(hsp24_t *sensor)
            {
                u8        outBuffer[64];
                s32       outLength = encodeEnableConfiguration(outBuffer);
                const u16 cmd       = outBuffer[FRAME_INTRAFRAME_CMD_OFFSET];
                sensor->mStreamWriter->write(outBuffer, outLength);
                nserial::frame_result_t frameResult;
                return waitForAck(sensor, cmd, frameResult);
            }

            EResult disableConfigMode(hsp24_t *sensor)
            {
                u8        outBuffer[64];
                const s32 outLength = encodeDisableConfiguration(outBuffer);
                const u16 cmd       = outBuffer[FRAME_INTRAFRAME_CMD_OFFSET];
                sensor->mStreamWriter->write(outBuffer, outLength);
                nserial::frame_result_t frameResult;
                return waitForAck(sensor, cmd, frameResult);
            }

            EResult getConfig(hsp24_t *sensor, RadarConfig &config)
            {
                EResult result = enableConfigMode(sensor);
                if (result != Success)
                    return result;

                u8        outBuffer[64];
                const s32 outLength = encodeReadParameters(outBuffer);
                sensor->mStreamWriter->write(outBuffer, outLength);
                const u16 cmd = outBuffer[FRAME_INTRAFRAME_CMD_OFFSET];

                nserial::frame_result_t frameResult;
                result = waitForAck(sensor, cmd, frameResult);
                if (result != Success)
                    return result;

                RadarParameters params;
                parseReadParametersAck(frameResult.frameStart, params);

                config.maximumDistanceGate       = params.maxDistanceGate;
                config.maximumMotionDistanceGate = params.motionGate;
                config.maximumRestDistanceGate   = params.restGate;
                for (s32 i = 0; i < 9; i++)
                {
                    config.motionSensitivityPerDistanceGate[i]     = params.motionSensitivity[i];
                    config.stationarySensitivityPerDistanceGate[i] = params.stationarySensitivity[i];
                }

                result = disableConfigMode(sensor);
                return result;
            }

            EResult getFirmwareVersion(hsp24_t *sensor, FirmwareVersion &version)
            {
                u8        outBuffer[64];
                const s32 outLength = encodeReadFirmwareVersion(outBuffer);
                sensor->mStreamWriter->write(outBuffer, outLength);
                const u16 cmd = outBuffer[FRAME_INTRAFRAME_CMD_OFFSET];

                nserial::frame_result_t frameResult;
                EResult                 result = waitForAck(sensor, cmd, frameResult);
                if (result != Success)
                    return result;

                parseReadFirmwareVersionAck(frameResult.frameStart, version.type, version.major, version.minor);
                return Success;
            }

            EResult setMode(hsp24_t *sensor, ERadarMode mode)
            {
                EResult result = enableConfigMode(sensor);
                if (result != Success)
                    return result;

                u8  outBuffer[64];
                s32 outLength = 0;
                if (mode == RadarMode_Engineering)
                {
                    outLength = encodeEnableEngineeringMode(outBuffer);
                }
                else
                {
                    outLength = encodeCloseEngineeringMode(outBuffer);
                }
                sensor->mStreamWriter->write(outBuffer, outLength);
                const u16 cmd = outBuffer[FRAME_INTRAFRAME_CMD_OFFSET];

                nserial::frame_result_t frameResult;
                result = waitForAck(sensor, cmd, frameResult);
                if (result != Success)
                    return result;

                result = disableConfigMode(sensor);
                return result;
            }

        }  // namespace nseeed
    }  // namespace nsensors
}  // namespace ncore
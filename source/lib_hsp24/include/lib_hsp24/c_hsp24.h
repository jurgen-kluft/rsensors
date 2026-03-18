#ifndef __ARDUINO_SENSORS_SEEED_HSP24_H__
#define __ARDUINO_SENSORS_SEEED_HSP24_H__
#include "rcore/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

#include "rsensors/c_frame_reader.h"

namespace ncore
{
    class reader_t;

    namespace nsensors
    {
        namespace nseeed
        {
            struct hsp24_t;
            hsp24_t* create_hsp24(ncore::reader_t* serial_reader);

            // Energy value for each moving distance gate
            struct MovementEnergy
            {
                void reset();
                u8   gate[9];
            };

            // Energy value for each stationary distance gate
            struct StationaryEnergy
            {
                void reset();
                u8   gate[9];
            };

            enum ERadarMode
            {
                RadarMode_Engineering = 0x01,
                RadarMode_Normal      = 0x02,
            };

            enum ETargetStatus
            {
                TargetStatusNone       = 0x00,
                TargetStatusMoving     = 0x01,
                TargetStatusStatic     = 0x02,
                TargetStatusBoth       = 0x03,
                TargetStatusFrameError = 0x04
            };

            inline bool isTargetDetected(ETargetStatus status)
            {
                return (status == TargetStatusMoving) || (status == TargetStatusStatic) || (status == TargetStatusBoth);
            }

            struct RadarStatus
            {
                ETargetStatus targetStatus;  // Target status of the radar
                ERadarMode    radarMode;     // Reporting mode (2) or engineering reporting mode (1)
                // ---- Basic Information ----
                u16 movementTargetDistance;    // Distance to the moving target (cm)
                u8  movementTargetEnergy;      // Energy of the moving target
                u16 stationaryTargetDistance;  // Distance to the static target (cm)
                u8  stationaryTargetEnergy;    // Energy of the static target
                u16 detectionDistance;         // Maximum detection range gate of the radar (cm)
                // ---- Engineering Information ----
                u8               maximumMovementDistanceDoor;
                u8               maximumRestingDistanceDoor;
                MovementEnergy   movementDistanceGateEnergy;    // Movement energy value for each distance gate
                StationaryEnergy stationaryDistanceGateEnergy;  // Stationary energy value for each distance gate
                u8               photosensitive;                // Photosensitive value 0-255
            };

            struct RadarParameters
            {
                u8  maxDistanceGate;
                u8  motionGate;
                u8  restGate;
                u8  motionSensitivity[9];
                u8  stationarySensitivity[9];
                u16 unoccupiedDuration;
            };

            struct RadarConfig
            {
                u8 maximumDistanceGate;
                u8 maximumMotionDistanceGate;
                u8 maximumRestDistanceGate;
                u8 motionSensitivityPerDistanceGate[9];
                u8 stationarySensitivityPerDistanceGate[9];
            };

            struct RadarMotionAndStationaryConfig
            {
                u32 maximumMotionDistanceGateValue;
                u32 maximumRestDistanceGateValue;
                u16 unoccupiedDurationValue;  // in seconds
            };

            struct RadarSensitityConfig
            {
                u8  distanceGate;       // Gate 0-8 (specify 0xFF to mean all gates)
                u32 motionSensitivity;  // Motion sensitivity value for the specified gate
                u32 restSensitivity;    // Rest sensitivity value for the specified gate
            };

            enum EResult
            {
                Success = 0,
                Fail    = -1,
                Timeout = -2,
            };

            enum EDistanceResolution
            {
                DistanceResolution_20cm = 0x01,
                DistanceResolution_75cm = 0x00,
            };

            struct FirmwareVersion
            {
                u16  type;                                    // 0x0001
                u16  major;                                   // 0x0107 = 1.07.22091516
                u32  minor;                                   // 0x22091516 = 22091516
                void toString(char* outStr, s32 outStrSize);  // Output format: "1.07.22091516"
            };

            s32 enterATMode(hsp24_t* sensor);
            s32 exitATMode(hsp24_t* sensor);
            s32 getVersion(hsp24_t* sensor);
            s32 setNetwork(hsp24_t* sensor, const char* ssid, const char* password);

            EResult getStatus(hsp24_t* sensor, RadarStatus& status);
            EResult enableConfigMode(hsp24_t* sensor);
            EResult disableConfigMode(hsp24_t* sensor);
            EResult getConfig(hsp24_t* sensor, RadarConfig& config);
            EResult getFirmwareVersion(hsp24_t* sensor, FirmwareVersion& version);
            EResult setMode(hsp24_t* sensor, ERadarMode mode);
            EResult setMaximumDistanceAndTiming(hsp24_t* sensor, RadarMotionAndStationaryConfig const& config);
            EResult setDistanceGateSensitity(hsp24_t* sensor, RadarSensitityConfig const& config);
            EResult setResolution(hsp24_t* sensor, EDistanceResolution resolution);
            EResult getResolution(hsp24_t* sensor, EDistanceResolution& resolution);

            EResult setBluetoothPassword(hsp24_t* sensor, const char password[6]);
            EResult getBluetoothPassword(hsp24_t* sensor, char* password);

            EResult rebootRadar(hsp24_t* sensor);
            EResult restoreFactorySettings(hsp24_t* sensor);

        }  // namespace nseeed
    }  // namespace nsensors
}  // namespace ncore

#endif  // __ARDUINO_SENSORS_RD03D_H__

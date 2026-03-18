#ifndef __ARDUINO_SENSORS_ENVIRONMENT_H__
#define __ARDUINO_SENSORS_ENVIRONMENT_H__
#include "rcore/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    namespace nenvironment
    {
        // Temperature unit enumeration.
        enum TempUnit
        {
            TempUnit_Celsius,
            TempUnit_Fahrenheit
        };

        // Altitude unit enumeration.
        enum AltitudeUnit
        {
            AltitudeUnit_Meters,
            AltitudeUnit_Feet
        };

        bool valid_temperature(f32 temperature);  // in °C
        bool valid_humidity(f32 humidity);        // in %
        bool valid_pressure(f32 pressure);        // in hPa

        bool valid_temperature(s8 temperature);  // in °C
        bool valid_humidity(u8 humidity);        // in %
        bool valid_pressure(u16 pressure);       // in hPa

        bool valid_co2(u16 co2);  // in ppm
        bool valid_lux(u16 co2);  // in lux

        // conversion from [°C] to [°F]
        inline f32 CelciusToFahrenheit(f32 celsius) { return (celsius * 9.0f / 5.0f) + 32.0f; }
        // conversion from [°F] to [°C]
        inline f32 FahrenheitToCelcius(f32 fahrenheit) { return (fahrenheit - 32.0f) * 5.0f / 9.0f; }

        // Calculate the altitude based on the pressure and temperature
        // in temptUnit.
        // @param pressure at the station in any units.
        // @param altUnit meters or feet. default=AltitudeUnit_Meters
        // @param referencePressure (usually pressure on MSL)
        //          in the same units as pressure. default=1013.25hPa (ISA)
        // @param outdoorTemp temperature at the station in tempUnit
        //          default=15°C (ISA)
        // @param temptUnit in °C or °F. default=TempUnit_Celsius
        // @return Calculated Altitude in altUnit.
        f32 Altitude(f32 pressure, AltitudeUnit altUnit = AltitudeUnit_Meters,
                     f32      referencePressure = 1013.25,  // [hPa] ....ISA value
                     f32      outdoorTemp       = 15,       // [°C] .... ISA value
                     TempUnit tempUnit          = TempUnit_Celsius);

        // Calculate the heatindex based on the humidity and temperature
        // in tempUnit.
        // The formula based on the Heat Index Equation of the US National Weather Service
        // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
        // @param temperature in tempUnit
        // @param humidity in percentage
        // @param temptUnit in °C or °F. default=TempUnit_Celsius
        // @return Calculated heatindex as f32 in TempUnit
        f32 HeatIndex(f32 temperature, f32 humidity, TempUnit tempUnit = TempUnit_Celsius);

        // Calculate the absolute humidity based on the relative humidity and temperature
        // in tempUnit.
        // the formula does work for values between -30°C and 35°C with 0.1°C precision
        // @param temperature in tempUnit
        // @param humidity in percentage
        // @param tempUnit in °C. default=TempUnit_Celsius
        // @return Calculated absolute humidity in grams/m³
        f32 AbsoluteHumidity(f32 temperature, f32 humidity, TempUnit tempUnit);

        // Convert current pressure to equivalent sea-level pressure.
        // @param altitude in altUnit.
        // @param temp in tempUnit.
        // @param pressure at the station in any units.
        // @param altUnit meters or feet. default=AltitudeUnit_Meters
        // @param tempUnit in °C or °F. default=TempUnit_Celsius
        // @return Equivalent pressure at sea level. The input pressure
        //          unit will determine the output
        //          pressure unit.
        f32 EquivalentSeaLevelPressure(f32 altitude, f32 temp, f32 pres, AltitudeUnit altUnit = AltitudeUnit_Meters, TempUnit tempUnit = TempUnit_Celsius);

        // Calculate the dew point based on the temperature in tempUnit
        // and humidity.
        // @param temp in tempUnit.
        // @param hum in %.
        // @param temptUnit in °C or °F. default=TempUnit_Celsius
        f32 DewPoint(f32 temp, f32 hum, TempUnit tempUnit = TempUnit_Celsius);

    }  // namespace nenvironment
}  // namespace ncore

#endif  // __ARDUINO_SENSORS_ENVIRONMENT_H__

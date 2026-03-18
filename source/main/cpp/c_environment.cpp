#include "rsensors/c_environment.h"

#include <math.h>

namespace ncore
{
    namespace nenvironment
    {
        /*
        EnvironmentCalculations.cpp

        Copyright (C) 2016  Tyler Glenn

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <http://www.gnu.org/licenses/>.

        Written: Dec 30 2015.
        Last Updated: Dec 23 2017.

        This header must be included in any derived code or copies of the code.

        */

        bool valid_temperature(f32 temperature)
        {
            if (isnan(temperature))
                return false;
            return (temperature > -100.0f && temperature < 100.0f);
        }

        bool valid_humidity(f32 humidity) { return (humidity >= 0.0f && humidity <= 100.0f); }
        bool valid_pressure(f32 pressure) { return (pressure > 300.0f && pressure < 1500.0f); }

        bool valid_temperature(s8 temperature) { return (temperature > -40 && temperature < 85); }
        bool valid_humidity(u8 humidity) { return (humidity <= 100); }
        bool valid_pressure(u16 pressure) { return (pressure > 300 && pressure < 1500); }

        bool valid_co2(u16 co2) { return (co2 > 0 && co2 < 10000); }
        bool valid_lux(u16 lux) { return (lux < 0xFFFF); }

#define hi_coeff1 -42.379f
#define hi_coeff2 2.04901523f
#define hi_coeff3 10.14333127f
#define hi_coeff4 -0.22475541f
#define hi_coeff5 -0.00683783f
#define hi_coeff6 -0.05481717f
#define hi_coeff7 0.00122874f
#define hi_coeff8 0.00085282f
#define hi_coeff9 -0.00000199f

        f32 Altitude(f32 pressure, AltitudeUnit altUnit, f32 referencePressure, f32 outdoorTemp, TempUnit tempUnit)
        {
            // Equation inverse to EquivalentSeaLevelPressure calculation.
            f32 altitude = NAN;
            if (!isnan(pressure) && !isnan(referencePressure) && !isnan(outdoorTemp))
            {
                if (tempUnit != TempUnit_Celsius)
                    outdoorTemp = FahrenheitToCelcius(outdoorTemp);

                altitude = pow(referencePressure / pressure, 0.190234f) - 1;
                altitude *= ((outdoorTemp + 273.15f) / 0.0065f);
                if (altUnit != AltitudeUnit_Meters)
                    altitude *= 3.28084f;
            }
            return altitude;
        }

        f32 AbsoluteHumidity(f32 temperature, f32 humidity, TempUnit tempUnit)
        {
            // taken from https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
            // precision is about 0.1°C in range -30 to 35°C
            // August-Roche-Magnus 	6.1094 exp(17.625 x T)/(T + 243.04)
            // Buck (1981) 		6.1121 exp(17.502 x T)/(T + 240.97)
            // reference https://www.eas.ualberta.ca/jdwilson/EAS372_13/Vomel_CIRES_satvpformulae.html
            f32       temp = NAN;
            const f32 mw   = 18.01534f;    // molar mass of water g/mol
            const f32 r    = 8.31447215f;  // Universal gas constant J/mol/K

            if (isnan(temperature) || isnan(humidity))
            {
                return NAN;
            }

            if (tempUnit != TempUnit_Celsius)
            {
                temperature = FahrenheitToCelcius(temperature); /*conversion to [°C]*/
            }

            temp = pow(2.718281828f, (17.67f * temperature) / (temperature + 243.5f));

            // return (6.112 * temp * humidity * 2.1674) / (273.15 + temperature); 	//simplified version
            return (6.112f * temp * humidity * mw) / ((273.15f + temperature) * r);  // long version
        }

        // FYI: https://ehp.niehs.nih.gov/1206273/ in detail this flow graph: https://ehp.niehs.nih.gov/wp-content/uploads/2013/10/ehp.1206273.g003.png
        f32 HeatIndex(f32 temperature, f32 humidity, TempUnit tempUnit)
        {
            f32 heatIndex(NAN);

            if (isnan(temperature) || isnan(humidity))
            {
                return heatIndex;
            }

            if (tempUnit == TempUnit_Celsius)
            {
                temperature = CelciusToFahrenheit(temperature);
            }

            // Using both Rothfusz and Steadman's equations
            // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
            if (temperature <= 40.0f)
            {
                heatIndex = temperature;  // first red block
            }
            else
            {
                heatIndex = 0.5f * (temperature + 61.0f + ((temperature - 68.0f) * 1.2f) + (humidity * 0.094f));  // calculate A -- from the official site, not the flow graph

                if (heatIndex >= 79.0f)
                {
                    /*
                     * calculate B
                     * the following calculation is optimized. Simply spoken, reduzed cpu-operations to minimize used ram and runtime.
                     * Check the correctness with the following link:
                     * http://www.wolframalpha.com/input/?source=nav&i=b%3D+x1+%2B+x2*T+%2B+x3*H+%2B+x4*T*H+%2B+x5*T*T+%2B+x6*H*H+%2B+x7*T*T*H+%2B+x8*T*H*H+%2B+x9*T*T*H*H
                     */
                    heatIndex = hi_coeff1 + (hi_coeff2 + hi_coeff4 * humidity + temperature * (hi_coeff5 + hi_coeff7 * humidity)) * temperature + (hi_coeff3 + humidity * (hi_coeff6 + temperature * (hi_coeff8 + hi_coeff9 * temperature))) * humidity;
                    // third red block
                    if ((humidity < 13.f) && (temperature >= 80.0f) && (temperature <= 112.0f))
                    {
                        heatIndex -= ((13.0f - humidity) * 0.25f) * sqrt((17.0f - abs(temperature - 95.0f)) * 0.05882f);
                    }  // fourth red block
                    else if ((humidity > 85.0f) && (temperature >= 80.0f) && (temperature <= 87.0f))
                    {
                        heatIndex += (0.02f * (humidity - 85.0f) * (87.0f - temperature));
                    }
                }
            }

            if (tempUnit == TempUnit_Celsius)
            {
                return (heatIndex - 32.0) * (5.0 / 9.0); /*conversion back to [°C]*/
            }
            else
            {
                return heatIndex;  // fifth red block
            }
        }

        f32 EquivalentSeaLevelPressure(f32 altitude, f32 temp, f32 pres, AltitudeUnit altUnit, TempUnit tempUnit)
        {
            f32 seaPress = NAN;
            if (!isnan(altitude) && !isnan(temp) && !isnan(pres))
            {
                if (tempUnit != TempUnit_Celsius)
                    temp = FahrenheitToCelcius(temp);

                if (altUnit != AltitudeUnit_Meters)
                    altitude *= 0.3048f; /*conversion to meters*/

                seaPress = (pres / pow(1 - ((0.0065f * altitude) / (temp + (0.0065f * altitude) + 273.15f)), 5.257f));
            }
            return seaPress;
        }

        f32 DewPoint(f32 temp, f32 hum, TempUnit tempUnit)
        {
            // Equations courtesy of Brian McNoldy from http://andrew.rsmas.miami.edu;
            f32 dewPoint = NAN;

            if (!isnan(temp) && !isnan(hum))
            {
                if (tempUnit == TempUnit_Celsius)
                {
                    dewPoint = 243.04f * (log(hum / 100.0f) + ((17.625f * temp) / (243.04f + temp))) / (17.625f - log(hum / 100.0f) - ((17.625f * temp) / (243.04f + temp)));
                }
                else
                {
                    f32 ctemp = FahrenheitToCelcius(temp); /*conversion to [°C]*/
                    dewPoint  = 243.04f * (log(hum / 100.0f) + ((17.625f * ctemp) / (243.04f + ctemp))) / (17.625f - log(hum / 100.0f) - ((17.625f * ctemp) / (243.04f + ctemp)));
                    dewPoint  = CelciusToFahrenheit(dewPoint); /*conversion back to [°F]*/
                }
            }

            return dewPoint;
        }
    }  // namespace nenvironment
}  // namespace ncore
#ifdef TARGET_ARDUINO

#    include "Arduino.h"
#    include "Wire.h"

namespace nbme280
{
    // BME280 - Driver class for Bosch Bme280 sensor
    //
    // Based on the data sheet provided by Bosch for
    // the Bme280 environmental sensor.
    //
    // https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors/bme280/

    /*
    BME280.cpp
    This code records data from the BME280 sensor and provides an API.
    This file is part of the Arduino BME280 library.
    Copyright (C) 2016   Tyler Glenn

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.   See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.   If not, see <http://www.gnu.org/licenses/>.

    Written: Dec 30 2015.
    Last Updated: Oct 07 2017.

    This header must be included in any derived code or copies of the code.

    Based on the data sheet provided by Bosch for the Bme280 environmental sensor,
    calibration code based on algorithms providedBosch, some unit conversations courtesy
    of www.endmemo.com, altitude equation courtesy of NOAA, and dew point equation
    courtesy of Brian McNoldy at http://andrew.rsmas.miami.edu.
     */

    class IReadWriteRegister
    {
    public:
        virtual bool WriteRegister(uint8_t addr, uint8_t data)                  = 0;
        virtual bool ReadRegister(uint8_t addr, uint8_t data[], uint8_t length) = 0;
    };

    enum EBME280Settings
    {
        CTRL_HUM_ADDR        = 0xF2,
        CTRL_MEAS_ADDR       = 0xF4,
        CONFIG_ADDR          = 0xF5,
        PRESS_ADDR           = 0xF7,
        TEMP_ADDR            = 0xFA,
        HUM_ADDR             = 0xFD,
        TEMP_DIG_ADDR        = 0x88,
        PRESS_DIG_ADDR       = 0x8E,
        HUM_DIG_ADDR1        = 0xA1,
        HUM_DIG_ADDR2        = 0xE1,
        ID_ADDR              = 0xD0,
        TEMP_DIG_LENGTH      = 6,
        PRESS_DIG_LENGTH     = 18,
        HUM_DIG_ADDR1_LENGTH = 1,
        HUM_DIG_ADDR2_LENGTH = 7,
        DIG_LENGTH           = 32,
        SENSOR_DATA_LENGTH   = 8,
    };

    enum TempUnit
    {
        TempUnit_Celsius,
        TempUnit_Fahrenheit
    };

    enum PresUnit
    {
        PresUnit_Pa,
        PresUnit_hPa,
        PresUnit_inHg,
        PresUnit_atm,
        PresUnit_bar,
        PresUnit_torr,
        PresUnit_psi
    };

    enum OSR
    {
        OSR_Off = 0,
        OSR_X1  = 1,
        OSR_X2  = 2,
        OSR_X4  = 3,
        OSR_X8  = 4,
        OSR_X16 = 5
    };

    enum Mode
    {
        Mode_Sleep  = 0,
        Mode_Forced = 1,
        Mode_Normal = 3
    };

    enum StandbyTime
    {
        StandbyTime_500us   = 0,
        StandbyTime_62500us = 1,
        StandbyTime_125ms   = 2,
        StandbyTime_250ms   = 3,
        StandbyTime_50ms    = 4,
        StandbyTime_1000ms  = 5,
        StandbyTime_10ms    = 6,
        StandbyTime_20ms    = 7
    };

    enum Filter
    {
        Filter_Off = 0,
        Filter_2   = 1,
        Filter_4   = 2,
        Filter_8   = 3,
        Filter_16  = 4
    };

    enum SpiEnable
    {
        SpiEnable_False = 0,
        SpiEnable_True  = 1
    };

    enum ChipModel
    {
        ChipModel_UNKNOWN = 0,
        ChipModel_BMP280  = 0x58,
        ChipModel_BME280  = 0x60
    };

    struct Settings
    {
        Settings(OSR _tosr = OSR_X1, OSR _hosr = OSR_X1, OSR _posr = OSR_X1, Mode _mode = Mode_Forced, StandbyTime _st = StandbyTime_1000ms, Filter _filter = Filter_Off, SpiEnable _se = SpiEnable_False)
            : tempOSR(_tosr)
            , humOSR(_hosr)
            , presOSR(_posr)
            , mode(_mode)
            , standbyTime(_st)
            , filter(_filter)
            , spiEnable(_se)
        {
        }

        OSR         tempOSR;
        OSR         humOSR;
        OSR         presOSR;
        Mode        mode;
        StandbyTime standbyTime;
        Filter      filter;
        SpiEnable   spiEnable;
    };

    class BME280
    {
    public:
        // Constructor used to create the class.
        // All parameters have default values.
        BME280();

        // Method used to initialize the class.
        bool begin(const Settings* settings, IReadWriteRegister* readWrite);

        // Read the temperature from the BME280 and return a float.
        float temp(TempUnit unit = TempUnit_Celsius);

        // Read the pressure from the BME280 and return a float with the specified unit.
        float pres(PresUnit unit = PresUnit_hPa);

        // Read the humidity from the BME280 and return a percentage as a float.
        float hum();

        // Read the data from the BME280 in the specified unit.
        bool read(float& pressure, float& temperature, float& humidity, TempUnit tempUnit = TempUnit_Celsius, PresUnit presUnit = PresUnit_hPa);

        // Write configuration to BME280, return true if successful.
        // Must be called from any child classes.
        bool Initialize(const Settings* settings);

        // Force a unfiltered measurement to populate the filter
        // buffer.
        void InitializeFilter();

        Settings m_settings;
        void     setSettings(const Settings& settings);

        IReadWriteRegister* m_readWriteReg;

        uint8_t   m_dig[32];
        ChipModel m_chip_model;

        bool m_initialized;

        // Calculates registers based on settings.
        void CalculateRegisters(uint8_t& ctrlHum, uint8_t& ctrlMeas, uint8_t& config);

        // Write the settings to the chip.
        void WriteSettings();

        // Read the the chip id data from the BME280, return true if
        // successful and the id matches a known value.
        bool ReadChipID();

        // Read the the trim data from the BME280, return true if
        // successful.
        bool ReadTrim();

        // Read the raw data from the BME280 into an array and return
        // true if successful.
        bool ReadData(int32_t data[SENSOR_DATA_LENGTH]);

        // Calculate the temperature from the BME280 raw data and
        // BME280 trim, return a float.
        float CalculateTemperature(int32_t raw, int32_t& t_fine, TempUnit unit = TempUnit_Celsius);

        // Calculate the humidity from the BME280 raw data and BME280
        // trim, return a float.
        float CalculateHumidity(int32_t raw, int32_t t_fine);

        // Calculate the pressure from the BME280 raw data and BME280
        // trim, return a float.
        float CalculatePressure(int32_t raw, int32_t t_fine, PresUnit unit = PresUnit_hPa);
    };

    BME280::BME280()
        : m_settings()
        , m_initialized(false)
    {
    }

    bool BME280::Initialize(const Settings* settings)
    {
        bool success(true);

        if (settings != nullptr)
            m_settings = *settings;

        success &= ReadChipID();

        if (success)
        {
            success &= ReadTrim();

            if (m_settings.filter != Filter_Off)
            {
                InitializeFilter();
            }

            WriteSettings();
        }

        m_initialized = success;

        return m_initialized;
    }

    void BME280::InitializeFilter()
    {
        // Force an unfiltered measurement to populate the filter buffer.
        // This fixes a bug that causes the first read to always be 28.82 °C 81732.34 hPa.
        Filter filter     = m_settings.filter;
        m_settings.filter = Filter_Off;

        WriteSettings();

        float dummy;
        read(dummy, dummy, dummy);

        m_settings.filter = filter;
    }

    bool BME280::ReadChipID()
    {
        uint8_t id[1];

        m_readWriteReg->ReadRegister(ID_ADDR, &id[0], 1);

        switch (id[0])
        {
            case ChipModel_BME280: m_chip_model = ChipModel_BME280; break;
            case ChipModel_BMP280: m_chip_model = ChipModel_BMP280; break;
            default: m_chip_model = ChipModel_UNKNOWN; return false;
        }

        return true;
    }

    void BME280::WriteSettings()
    {
        uint8_t ctrlHum, ctrlMeas, config;

        CalculateRegisters(ctrlHum, ctrlMeas, config);

        m_readWriteReg->WriteRegister(CTRL_HUM_ADDR, ctrlHum);
        m_readWriteReg->WriteRegister(CTRL_MEAS_ADDR, ctrlMeas);
        m_readWriteReg->WriteRegister(CONFIG_ADDR, config);
    }

    void BME280::setSettings(const Settings& settings)
    {
        m_settings = settings;
        WriteSettings();
    }

    bool BME280::begin(const Settings* settings, IReadWriteRegister* readWrite)
    {
        m_readWriteReg = readWrite;
        return Initialize(settings);
    }

    void BME280::CalculateRegisters(uint8_t& ctrlHum, uint8_t& ctrlMeas, uint8_t& config)
    {
        // ctrl_hum register. (ctrl_hum[2:0] = Humidity oversampling rate.)
        ctrlHum = (uint8_t)m_settings.humOSR;
        // ctrl_meas register. (ctrl_meas[7:5] = temperature oversampling rate, ctrl_meas[4:2] = pressure oversampling rate, ctrl_meas[1:0] = mode.)
        ctrlMeas = ((uint8_t)m_settings.tempOSR << 5) | ((uint8_t)m_settings.presOSR << 2) | (uint8_t)m_settings.mode;
        // config register. (config[7:5] = standby time, config[4:2] = filter, ctrl_meas[0] = spi enable.)
        config = ((uint8_t)m_settings.standbyTime << 5) | ((uint8_t)m_settings.filter << 2) | (uint8_t)m_settings.spiEnable;
    }

    bool BME280::ReadTrim()
    {
        uint8_t ord(0);
        bool    success = true;

        // Temp. Dig
        success &= m_readWriteReg->ReadRegister(TEMP_DIG_ADDR, &m_dig[ord], TEMP_DIG_LENGTH);
        ord += TEMP_DIG_LENGTH;

        // Pressure Dig
        success &= m_readWriteReg->ReadRegister(PRESS_DIG_ADDR, &m_dig[ord], PRESS_DIG_LENGTH);
        ord += PRESS_DIG_LENGTH;

        // Humidity Dig 1
        success &= m_readWriteReg->ReadRegister(HUM_DIG_ADDR1, &m_dig[ord], HUM_DIG_ADDR1_LENGTH);
        ord += HUM_DIG_ADDR1_LENGTH;

        // Humidity Dig 2
        success &= m_readWriteReg->ReadRegister(HUM_DIG_ADDR2, &m_dig[ord], HUM_DIG_ADDR2_LENGTH);
        ord += HUM_DIG_ADDR2_LENGTH;

#    ifdef TARGET_DEBUG
        Serial.print("Dig: ");
        for (int i = 0; i < 32; ++i)
        {
            Serial.print(m_dig[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
#    endif

        return success && ord == DIG_LENGTH;
    }

    bool BME280::ReadData(int32_t data[SENSOR_DATA_LENGTH])
    {
        bool    success;
        uint8_t buffer[SENSOR_DATA_LENGTH];

        // For forced mode we need to write the mode to BME280 register before reading
        if (m_settings.mode == Mode_Forced)
        {
            WriteSettings();
        }

        // Registers are in order. So we can start at the pressure register and read 8 bytes.
        success = m_readWriteReg->ReadRegister(PRESS_ADDR, buffer, SENSOR_DATA_LENGTH);

        for (int i = 0; i < SENSOR_DATA_LENGTH; ++i)
        {
            data[i] = static_cast<int32_t>(buffer[i]);
        }

#    ifdef TARGET_DEBUG
        Serial.print("Data: ");
        for (int i = 0; i < 8; ++i)
        {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
#    endif

        return success;
    }

    float BME280::CalculateTemperature(int32_t raw, int32_t& t_fine, TempUnit unit)
    {
        // Code based on calibration algorthim provided by Bosch.
        int32_t  var1, var2, final;
        uint16_t dig_T1 = (m_dig[1] << 8) | m_dig[0];
        int16_t  dig_T2 = (m_dig[3] << 8) | m_dig[2];
        int16_t  dig_T3 = (m_dig[5] << 8) | m_dig[4];
        var1            = ((((raw >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
        var2            = (((((raw >> 4) - ((int32_t)dig_T1)) * ((raw >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
        t_fine          = var1 + var2;
        final           = (t_fine * 5 + 128) >> 8;
        return unit == TempUnit_Celsius ? final / 100.0 : final / 100.0 * 9.0 / 5.0 + 32.0;
    }

    float BME280::CalculateHumidity(int32_t raw, int32_t t_fine)
    {
        // Code based on calibration algorthim provided by Bosch.
        int32_t var1;
        uint8_t dig_H1 = m_dig[24];
        int16_t dig_H2 = (m_dig[26] << 8) | m_dig[25];
        uint8_t dig_H3 = m_dig[27];
        int16_t dig_H4 = (m_dig[28] << 4) | (0x0F & m_dig[29]);
        int16_t dig_H5 = (m_dig[30] << 4) | ((m_dig[29] >> 4) & 0x0F);
        int8_t  dig_H6 = m_dig[31];

        var1 = (t_fine - ((int32_t)76800));
        var1 = (((((raw << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var1)) + ((int32_t)16384)) >> 15) *
                (((((((var1 * ((int32_t)dig_H6)) >> 10) * (((var1 * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
        var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
        var1 = (var1 < 0 ? 0 : var1);
        var1 = (var1 > 419430400 ? 419430400 : var1);
        return ((uint32_t)(var1 >> 12)) / 1024.0;
    }

    float BME280::CalculatePressure(int32_t raw, int32_t t_fine, PresUnit unit)
    {
        // Code based on calibration algorthim provided by Bosch.
        int64_t var1, var2, pressure;
        float   final;

        uint16_t dig_P1 = (m_dig[7] << 8) | m_dig[6];
        int16_t  dig_P2 = (m_dig[9] << 8) | m_dig[8];
        int16_t  dig_P3 = (m_dig[11] << 8) | m_dig[10];
        int16_t  dig_P4 = (m_dig[13] << 8) | m_dig[12];
        int16_t  dig_P5 = (m_dig[15] << 8) | m_dig[14];
        int16_t  dig_P6 = (m_dig[17] << 8) | m_dig[16];
        int16_t  dig_P7 = (m_dig[19] << 8) | m_dig[18];
        int16_t  dig_P8 = (m_dig[21] << 8) | m_dig[20];
        int16_t  dig_P9 = (m_dig[23] << 8) | m_dig[22];

        var1 = (int64_t)t_fine - 128000;
        var2 = var1 * var1 * (int64_t)dig_P6;
        var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
        var2 = var2 + (((int64_t)dig_P4) << 35);
        var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
        var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
        if (var1 == 0)
        {
            return NAN;
        }  // Don't divide by zero.
        pressure = 1048576 - raw;
        pressure = (((pressure << 31) - var2) * 3125) / var1;
        var1     = (((int64_t)dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
        var2     = (((int64_t)dig_P8) * pressure) >> 19;
        pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

        final = ((uint32_t)pressure) / 256.0;

        // Conversion units courtesy of www.endmemo.com.
        switch (unit)
        {
            case PresUnit_hPa: /* hPa */ final /= 100.0; break;
            case PresUnit_inHg:           /* inHg */
                final /= 3386.3752577878; /* final pa * 1inHg/3386.3752577878Pa */
                break;
            case PresUnit_atm:            /* atm */
                final /= 101324.99766353; /* final pa * 1 atm/101324.99766353Pa */
                break;
            case PresUnit_bar:     /* bar */
                final /= 100000.0; /* final pa * 1 bar/100kPa */
                break;
            case PresUnit_torr:           /* torr */
                final /= 133.32236534674; /* final pa * 1 torr/133.32236534674Pa */
                break;
            case PresUnit_psi:           /* psi */
                final /= 6894.744825494; /* final pa * 1psi/6894.744825494Pa */
                break;
            default: /* Pa (case: 0) */ break;
        }
        return final;
    }

    float BME280::temp(TempUnit unit)
    {
        int32_t data[8];
        int32_t t_fine;
        if (!ReadData(data))
        {
            return NAN;
        }
        uint32_t rawTemp = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
        return CalculateTemperature(rawTemp, t_fine, unit);
    }

    float BME280::pres(PresUnit unit)
    {
        int32_t data[8];
        int32_t t_fine;
        if (!ReadData(data))
        {
            return NAN;
        }
        uint32_t rawTemp     = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
        uint32_t rawPressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        CalculateTemperature(rawTemp, t_fine);
        return CalculatePressure(rawPressure, t_fine, unit);
    }

    float BME280::hum()
    {
        int32_t data[8];
        int32_t t_fine;
        if (!ReadData(data))
        {
            return NAN;
        }
        uint32_t rawTemp     = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
        uint32_t rawHumidity = (data[6] << 8) | data[7];
        CalculateTemperature(rawTemp, t_fine);
        return CalculateHumidity(rawHumidity, t_fine);
    }

    bool BME280::read(float& pressure, float& temp, float& humidity, TempUnit tempUnit, PresUnit presUnit)
    {
        int32_t data[8];
        int32_t t_fine;
        if (!ReadData(data))
        {
            pressure = temp = humidity = NAN;
            return false;
        }
        uint32_t rawPressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        uint32_t rawTemp     = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
        uint32_t rawHumidity = (data[6] << 8) | data[7];
        temp                 = CalculateTemperature(rawTemp, t_fine, tempUnit);
        pressure             = CalculatePressure(rawPressure, t_fine, presUnit);
        humidity             = CalculateHumidity(rawHumidity, t_fine);
        return true;
    }

    class Bme280ReadWriteI2CRegister : public IReadWriteRegister
    {
    public:
        Bme280ReadWriteI2CRegister()
            : m_bme280Addr(0)
        {
        }

        uint8_t m_bme280Addr;

        virtual bool WriteRegister(uint8_t addr, uint8_t data);
        virtual bool ReadRegister(uint8_t addr, uint8_t data[], uint8_t length);
    };

    bool Bme280ReadWriteI2CRegister::WriteRegister(uint8_t addr, uint8_t data)
    {
        Wire.beginTransmission(m_bme280Addr);
        Wire.write(addr);
        Wire.write(data);
        Wire.endTransmission();

        // TODO: Check return values from wire calls.
        return true;
    }

    bool Bme280ReadWriteI2CRegister::ReadRegister(uint8_t addr, uint8_t data[], uint8_t length)
    {
        uint8_t nBytesRead(0);

        Wire.beginTransmission(m_bme280Addr);
        Wire.write(addr);
        Wire.endTransmission();

        Wire.requestFrom(m_bme280Addr, length);
        while (Wire.available())
        {
            data[nBytesRead++] = Wire.read();
        }

        return nBytesRead == length;
    }

}  // namespace nbme280

#    include "lib_bme280/c_bme280.h"
#    include "rcore/c_malloc.h"

namespace ncore
{
    namespace nsensors
    {
        nbme280::BME280 gBme280;
        nbme280::Bme280ReadWriteI2CRegister gBme280Rw;

        bool initBME280(u8 i2c_address)
        {
            gBme280Rw.m_bme280Addr = i2c_address;
            while (!gBme280.begin(nullptr, &gBme280Rw))
            {
                Serial.println("BME280 init failed, retrying...");
                delay(1000);
            }

            switch (gBme280.m_chip_model)
            {
                case nbme280::ChipModel_BME280: Serial.println("Found BME280 sensor! "); break;
                case nbme280::ChipModel_BMP280: Serial.println("Found BMP280 sensor! "); break;
                default: Serial.println("Error unknown sensor! "); return false;
            }
            return true;
        }

        bool updateBME280(f32& outPressure, f32& outTemperature, f32& outHumidity)
        {
            if (!gBme280.read(outPressure, outTemperature, outHumidity))
            {
                outPressure    = 0;
                outTemperature = -100.0f;
                outHumidity    = 0;
                return false;
            }
            return true;
        }

        bool updateBME280(u16& outPressure, s8& outTemperature, u16& outHumidity)
        {
            f32 outPressureF;
            f32 outTemperatureF;
            f32 outHumidityF;
            if (!gBme280.read(outPressureF, outTemperatureF, outHumidityF))
            {
                outPressure    = 0;
                outTemperature = -100;
                outHumidity    = 0;
                return false;
            }

            outPressure    = static_cast<u16>(outPressureF);
            outTemperature = static_cast<s8>(outTemperatureF);
            outHumidity    = static_cast<u16>(outHumidityF);
            return true;
        }

    }  // namespace nsensors

}  // namespace ncore

#else

#    include "rsensors/c_bme280.h"

namespace ncore
{
    namespace nsensors
    {
        bool initBME280(u8 i2c_address) { return true; }

        bool updateBME280(f32& outPressure, f32& outTemperature, f32& outHumidity)
        {
            outPressure    = 1024.0f;
            outTemperature = 25.0f;
            outHumidity    = 49.f;
            return true;
        }

        bool updateBME280(u16& outPressure, s8& outTemperature, u16& outHumidity)
        {
            outPressure    = 1024;
            outTemperature = 25;
            outHumidity    = 49;
            return true;
        }

    }  // namespace nsensors
}  // namespace ncore

#endif
#include "rcore/c_target.h"
#include "rsensors/c_environment.h"

#include "cunittest/cunittest.h"

using namespace ncore;
using namespace ncore::nenvironment;

UNITTEST_SUITE_BEGIN(environment)
{
	UNITTEST_FIXTURE(temperature_conversion)
	{
		UNITTEST_FIXTURE_SETUP() {}
		UNITTEST_FIXTURE_TEARDOWN() {}

		UNITTEST_TEST(test_f_to_c)
		{
			f32 celsius = 0.0f;
			f32 fahrenheit = CelciusToFahrenheit(celsius);
			CHECK_EQUAL(fahrenheit, 32.0f);

			celsius = 100.0f;
			fahrenheit = CelciusToFahrenheit(celsius);
			CHECK_EQUAL(fahrenheit, 212.0f);
		}

		UNITTEST_TEST(test_c_to_f)
		{
			f32 fahrenheit = 32.0f;
			f32 celsius = FahrenheitToCelcius(fahrenheit);
			CHECK_EQUAL(celsius, 0.0f);

			fahrenheit = 212.0f;
			celsius = FahrenheitToCelcius(fahrenheit);
			CHECK_EQUAL(celsius, 100.0f);
		}
    }

	UNITTEST_FIXTURE(altitude_calculation)
	{
		UNITTEST_TEST(test_altitude_meters)
		{
			f32 pressure = 1013.25f; // hPa
			f32 altitude = Altitude(pressure, AltitudeUnit_Meters);
			CHECK_EQUAL(altitude, 0.0f); // At sea level

			pressure = 900.0f; // hPa
			altitude = Altitude(pressure, AltitudeUnit_Meters);
			CHECK(altitude > 0.0f); // Should be above sea level
		}
	}

	UNITTEST_FIXTURE(humidity_calculation)
	{
		UNITTEST_TEST(test_heat_index)
		{
			f32 temperature = 30.0f; // °C
			f32 humidity = 70.0f; // %
			f32 heatIndex = HeatIndex(temperature, humidity, TempUnit_Celsius);
			CHECK(heatIndex > temperature); // Heat index should be higher than temperature
		}

		UNITTEST_TEST(test_absolute_humidity)
		{
			f32 temperature = 20.0f; // °C
			f32 humidity = 50.0f; // %
			f32 absHumidity = AbsoluteHumidity(temperature, humidity, TempUnit_Celsius);
			CHECK(absHumidity > 0.0f); // Absolute humidity should be positive
		}
	}

	UNITTEST_FIXTURE(sea_level_pressure)
	{
		UNITTEST_TEST(test_equivalent_sea_level_pressure)
		{
			f32 altitude = 0.0f; // meters
			f32 temp = 15.0f; // °C
			f32 pressure = 1013.25f; // hPa
			f32 seaLevelPressure = EquivalentSeaLevelPressure(altitude, temp, pressure, AltitudeUnit_Meters, TempUnit_Celsius);
			CHECK_EQUAL(seaLevelPressure, 1013.25f); // At sea level, pressure should remain the same
		}
	}

	UNITTEST_FIXTURE(dew_point_calculation)
	{
		UNITTEST_TEST(test_dew_point)
		{
			f32 temperature = 25.0f; // °C
			f32 humidity = 60.0f; // %
			f32 dewPoint = DewPoint(temperature, humidity, TempUnit_Celsius);
			CHECK(dewPoint < temperature); // Dew point should be less than temperature
		}
	}

}
UNITTEST_SUITE_END

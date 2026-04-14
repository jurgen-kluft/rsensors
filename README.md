# rsensors

Arduino Sensor Library supporting the following sensors:

- [x] SCD41 (CO2, Temperature, Humidity)
- [x] BH1750 (Light intensity)
- [x] BME280 (Pressure, Temperature, Humidity)
- [x] HMMD (Human mmWave sensor)
- [x] RD03D (AI Thinker mmWave sensor)
- [x] HSP24 (Human Static Presence sensor (Xiao Seeed))
- [ ] MG58F18 (Radar sensor from MagorTek (cannot make it work))
- [x] SC7A20H (3-axis accelerometer)

## Frame reader

A configurable frame reader is included to read and parse frames from serial streams, HSP24 is currently the only sensor using this feature.

We also have a unittest for the frame reader in `source/test/cpp/test_frame_reader.cpp`.
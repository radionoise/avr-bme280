# BME280 humidity, pressure and temperature sensor library for AVR microcontroller

### Overview
This library was tested on ATmega 16 @ 8 MHz.

### Usage
You have to read sensor's individual calibration data and pass it every time when fetching readings.

### Example
```
Bme280CalibrationData calibrationData;
result = bme280ReadCalibrationData(&calibrationData);

if (result == BME280_OK) {
    Bme280Data data;
    result = bme280ReadData(BME280_OSS_1, BME280_OSS_1, BME280_OSS_1, &data, &calibrationData);

    if (result == BME280_OK) {
        float temperature = data.temperatureC;
        float humidity = data.humidityPercent
        long pressure = data.pressurePa;
    }
}
```
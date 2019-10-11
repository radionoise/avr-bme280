/*
 * BME280 humidity, pressure and temperature sensor library for AVR.
 * Version 1.0.0
 */

#ifndef BME280_BME280_H
#define BME280_BME280_H

#include <stdint-gcc.h>
#include <stdlib.h>

#ifndef BME280_ADDRESS
#error "BME280_ADDRESS is not defined. Use 0x77 when SDO connected to VCC and 0x76 when SDO connected to GND"
#endif

/*
 * All functions return 16-bit error code
 * [7:0] represents unexpected i2c status
 * [11:8] represents low-level error code like failed sending start condition, or sending register address
 * [15:12] represents hig-level error code like writing to ctrl_meas register failed
 */
#define BME280_OK 0

#define BME280_ERROR_LOW_START 1
#define BME280_ERROR_LOW_SEND_SLAVE_ADDR_WRITE 2
#define BME280_ERROR_LOW_SEND_REG_ADDR 3
#define BME280_ERROR_LOW_REPEATED_START 4
#define BME280_ERROR_LOW_SEND_SLAVE_ADDR_READ 5
#define BME280_ERROR_LOW_RECEIVE_DATA_ACK 6
#define BME280_ERROR_LOW_RECEIVE_DATA_NO_ACK 7
#define BME280_ERROR_LOW_SEND_REG_DATA 8

#define BME280_ERROR_HIGH_RESET_WRITE_RESET_REG 1
#define BME280_ERROR_HIGH_RESET_READ_STATUS_REG 2

#define BME280_ERROR_HIGH_READ_CAL_READ_BATCH_1 3
#define BME280_ERROR_HIGH_READ_CAL_READ_SINGLE 4
#define BME280_ERROR_HIGH_READ_CAL_READ_BATCH_2 5

#define BME280_ERROR_HIGH_READ_MEAS_WRITE_CTRLHUM_REG 6
#define BME280_ERROR_HIGH_READ_MEAS_WRITE_CTRLMEAS_REG 7
#define BME280_ERROR_HIGH_READ_MEAS_READ_STATUS_REG 8
#define BME280_ERROR_HIGH_READ_MEAS_READ_MEAS_REG 9

/*
 * Measurement value placeholder
 */
#define BME280_MEASUREMENT_NO_DATA 0xFFFF
#define BME280_MEASUREMENT_ERROR 0xFFFE

/*
 * Oversampling settings. Passing 0 to any measurement disables it
 */
#define BME280_OSS_DISABLED 0
#define BME280_OSS_1 0x1
#define BME280_OSS_2 0x2
#define BME280_OSS_4 0x3
#define BME280_OSS_8 0x4
#define BME280_OSS_16 0x5

typedef struct Bme280CalibrationData {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} Bme280CalibrationData;

typedef struct Bme280Data {
    float temperatureC;
    float humidityPercent;
    uint32_t pressurePa;
} Bme280Data;

uint16_t bme280Reset();
uint16_t bme280ReadCalibrationData(Bme280CalibrationData *data);
uint16_t bme280ReadData(uint8_t oversamplingT, uint8_t oversamplingH, uint8_t oversamplingP, Bme280Data *data, Bme280CalibrationData *calibrationData);
float bme280ConvertPressurePaToMmHg(uint32_t pressurePa);

#endif //BME280_BME280_H

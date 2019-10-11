#include <util/delay.h>
#include "bme280.h"
#include "i2c.h"

#define REG_ADDR_CTRL_MEAS 0xF4
#define REG_ADDR_CTRL_HUM 0xF2
#define REG_ADDR_STATUS 0xF3
#define REG_ADDR_RESET 0xE0

#define REG_VAL_RESET 0xB6

#define littleEndian8bitTo16Bit(firstByte, secondByte) (secondByte << 8) | firstByte

volatile int32_t t_fine;

uint16_t prepare() {
    i2cInit();

    i2cStart();
    if (i2cGetStatus() != I2C_STATUS_START_TRANSMITTED) {
        return (BME280_ERROR_LOW_START << 8) | i2cGetStatus();
    }

    i2cSendSlaveAddress(BME280_ADDRESS, I2C_WRITE);
    if (i2cGetStatus() != I2C_STATUS_ADDRESS_WRITE_ACK_RECEIVED) {
        return (BME280_ERROR_LOW_SEND_SLAVE_ADDR_WRITE << 8) | i2cGetStatus();
    }

    return BME280_OK;
}

uint16_t prepareRead(uint8_t regAddr) {
    uint16_t result = prepare();

    if (result != BME280_OK) {
        return result;
    }

    i2cSendData(regAddr);
    if (i2cGetStatus() != I2C_STATUS_DATA_WRITE_ACK_RECEIVED) {
        return (BME280_ERROR_LOW_SEND_REG_ADDR << 8) | i2cGetStatus();
    }

    i2cStart();
    if (i2cGetStatus() != I2C_STATUS_REPEATED_START_TRANSMITTED) {
        return (BME280_ERROR_LOW_REPEATED_START << 8) | i2cGetStatus();
    }

    i2cSendSlaveAddress(BME280_ADDRESS, I2C_READ);
    if (i2cGetStatus() != I2C_STATUS_ADDRESS_READ_ACK_SENT) {
        return (BME280_ERROR_LOW_SEND_SLAVE_ADDR_READ << 8) | i2cGetStatus();
    }

    return BME280_OK;
}

uint16_t readData(uint8_t regAddr, uint8_t *data) {
    uint16_t result = prepareRead(regAddr);

    if (result != BME280_OK) {
        return result;
    }

    uint8_t receivedData = i2cReadDataNotAck();
    if (i2cGetStatus() != I2C_STATUS_DATA_READ_ACK_NOT_SENT) {
        return (BME280_ERROR_LOW_RECEIVE_DATA_NO_ACK << 8) | i2cGetStatus();
    }

    *data = receivedData;

    i2cStop();

    return BME280_OK;
}

uint16_t readDataBatch(uint8_t regAddr, uint8_t *data, uint16_t size) {
    uint16_t result = prepareRead(regAddr);
    uint8_t receivedData = 0;

    if (result != BME280_OK) {
        return result;
    }

    for (uint16_t idx = 0; idx < size; idx++) {
        if (idx < size - 1) {
            receivedData = i2cReadDataAck();
            if (i2cGetStatus() != I2C_STATUS_DATA_READ_ACK_SENT) {
                return (BME280_ERROR_LOW_RECEIVE_DATA_ACK << 8) | i2cGetStatus();
            }
        } else {
            receivedData = i2cReadDataNotAck();
            uint8_t status = i2cGetStatus();

            i2cStop();

            if (status != I2C_STATUS_DATA_READ_ACK_NOT_SENT) {
                return (BME280_ERROR_LOW_RECEIVE_DATA_NO_ACK << 8) | status;
            }
        }

        data[idx] = receivedData;
    }

    return BME280_OK;
}

uint16_t writeData(uint8_t regAddr, uint8_t data) {
    uint16_t result = prepare();

    if (result != BME280_OK) {
        return result;
    }

    i2cSendData(regAddr);
    if (i2cGetStatus() != I2C_STATUS_DATA_WRITE_ACK_RECEIVED) {
        return (BME280_ERROR_LOW_SEND_REG_ADDR << 8) | i2cGetStatus();
    }

    i2cSendData(data);
    if (i2cGetStatus() != I2C_STATUS_DATA_WRITE_ACK_RECEIVED) {
        return (BME280_ERROR_LOW_SEND_REG_DATA << 8) | i2cGetStatus();
    }

    i2cStop();

    return BME280_OK;
}

uint16_t bme280Reset() {
    uint16_t result = writeData(REG_ADDR_RESET, REG_VAL_RESET);

    if (result != BME280_OK) {
        return (BME280_ERROR_HIGH_RESET_WRITE_RESET_REG << 12) | result;
    }

    uint8_t status = 0xFF;

    while (status & 0x1) {
        result = readData(REG_ADDR_STATUS, &status);

        if (result != BME280_OK) {
            return (BME280_ERROR_HIGH_RESET_READ_STATUS_REG << 12) | result;
        }

        _delay_ms(1);
    }

    return BME280_OK;
}

uint16_t bme280ReadCalibrationData(Bme280CalibrationData *data) {
    uint8_t buf[24];
    uint16_t result = readDataBatch(0x88, (uint8_t *) &buf, sizeof(buf)); // first calibration register address
    if (result != BME280_OK) {
        return (BME280_ERROR_HIGH_READ_CAL_READ_BATCH_1 << 12) | result;
    }

    data->dig_T1 = littleEndian8bitTo16Bit(buf[0], buf[1]);
    data->dig_T2 = littleEndian8bitTo16Bit(buf[2], buf[3]);
    data->dig_T3 = littleEndian8bitTo16Bit(buf[4], buf[5]);
    data->dig_P1 = littleEndian8bitTo16Bit(buf[6], buf[7]);
    data->dig_P2 = littleEndian8bitTo16Bit(buf[8], buf[9]);
    data->dig_P3 = littleEndian8bitTo16Bit(buf[10], buf[11]);
    data->dig_P4 = littleEndian8bitTo16Bit(buf[12], buf[13]);
    data->dig_P5 = littleEndian8bitTo16Bit(buf[14], buf[15]);
    data->dig_P6 = littleEndian8bitTo16Bit(buf[16], buf[17]);
    data->dig_P7 = littleEndian8bitTo16Bit(buf[18], buf[19]);
    data->dig_P8 = littleEndian8bitTo16Bit(buf[20], buf[21]);
    data->dig_P9 = littleEndian8bitTo16Bit(buf[22], buf[23]);

    uint8_t temp = 0;
    result = readData(0xA1, &temp); // this one is the gapped register
    if (result != BME280_OK) {
        return (BME280_ERROR_HIGH_READ_CAL_READ_SINGLE << 12) | result;
    }

    data->dig_H1 = temp;

    result = readDataBatch(0xE1, (uint8_t *) &buf, 7); // reading remaining 7 registers
    if (result != BME280_OK) {
        return (BME280_ERROR_HIGH_READ_CAL_READ_BATCH_2 << 12) | result;
    }

    data->dig_H2 = littleEndian8bitTo16Bit(buf[0], buf[1]);
    data->dig_H3 = buf[2];
    data->dig_H4 = (buf[3] << 4) | (buf[4] & 0xF);
    data->dig_H5 = (buf[5] << 4) | (buf[4] >> 4);
    data->dig_H6 = buf[6];

    return BME280_OK;
}

/*
 * Code from BME280 datasheet.
 * Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC
 */
int32_t calculateTemperature(uint32_t adcRawTemp, Bme280CalibrationData *calibrationData) {
    int32_t var1, var2, T;
    var1 = ((((adcRawTemp >> 3) - ((int32_t)calibrationData->dig_T1 << 1))) * ((int32_t)calibrationData->dig_T2)) >> 11;
    var2 = (((((adcRawTemp >> 4) - ((int32_t)calibrationData->dig_T1)) * ((adcRawTemp >> 4) - ((int32_t)calibrationData->dig_T1))) >> 12) *
            ((int32_t)calibrationData->dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return T;
}

/*
 * Code from BME280 datasheet.
 * Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
 * Output value of “47445” represents 47445/1024 = 46.333 %RH
 */
uint32_t calculateHumidity(uint32_t adcRawTemp, Bme280CalibrationData *calibrationData) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t) 76800));
    v_x1_u32r = (((((adcRawTemp << 14) - (((int32_t) calibrationData->dig_H4) << 20) - (((int32_t) calibrationData->dig_H5) * v_x1_u32r)) +
                   ((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) calibrationData->dig_H6)) >> 10) * (((v_x1_u32r *
            ((int32_t) calibrationData->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                                              ((int32_t) calibrationData->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) calibrationData->dig_H1)) >> 4)); v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t) (v_x1_u32r >> 12);
}

/*
 * Code from BME280 datasheet.
 * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
 * Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 */
uint32_t calculatePressure(uint32_t adcRawPress, Bme280CalibrationData *calibrationData) {
    int64_t var1, var2, p;
    var1 = ((int64_t) t_fine) - 128000;
    var2 = var1 * var1 * (int64_t) calibrationData->dig_P6;
    var2 = var2 + ((var1*(int64_t) calibrationData->dig_P5) << 17);
    var2 = var2 + (((int64_t) calibrationData->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t) calibrationData->dig_P3) >> 8) + ((var1 * (int64_t) calibrationData->dig_P2) << 12);
    var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) calibrationData->dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }

    p = 1048576 - adcRawPress;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t) calibrationData->dig_P9) * (p >> 13) * (p >> 13)) >> 25; var2 = (((int64_t) calibrationData->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t) calibrationData->dig_P7) << 4);

    return (uint32_t) p;
}

void setMeasurementError(Bme280Data *data) {
    data->temperatureC = BME280_MEASUREMENT_ERROR;
    data->humidityPercent = BME280_MEASUREMENT_ERROR;
    data->pressurePa = BME280_MEASUREMENT_ERROR;
}

uint16_t bme280ReadData(uint8_t oversamplingT, uint8_t oversamplingH, uint8_t oversamplingP, Bme280Data *data, Bme280CalibrationData *calibrationData) {
    uint8_t ctrlMeassReg = 0x1 | (oversamplingP << 2) | (oversamplingT << 5);
    uint8_t ctrlHumReg = oversamplingH;
    uint16_t result = writeData(REG_ADDR_CTRL_HUM, ctrlHumReg);

    if (result != BME280_OK) {
        setMeasurementError(data);
        return (BME280_ERROR_HIGH_READ_MEAS_WRITE_CTRLHUM_REG << 12) | result;
    }

    result = writeData(REG_ADDR_CTRL_MEAS, ctrlMeassReg);

    if (result != BME280_OK) {
        setMeasurementError(data);
        return (BME280_ERROR_HIGH_READ_MEAS_WRITE_CTRLMEAS_REG << 12) | result;
    }

    uint8_t status = 0xFF;
    _delay_ms(1); // reading status register immediately after writing to mode results in status = 0
    while (status & 0x8) {
        result = readData(REG_ADDR_STATUS, &status);

        if (result != BME280_OK) {
            setMeasurementError(data);
            return (BME280_ERROR_HIGH_READ_MEAS_READ_STATUS_REG << 12) | result;
        }

        _delay_ms(1);
    }

    uint8_t buf[8];
    result = readDataBatch(0xF7, (uint8_t *) &buf, sizeof(buf));
    if (result != BME280_OK) {
        setMeasurementError(data);
        return (BME280_ERROR_HIGH_READ_MEAS_READ_MEAS_REG << 12) | result;
    }

    uint32_t rawTemp = ((uint32_t) buf[3] << 12) | ((uint32_t) buf[4] << 4) | ((uint32_t) buf[5] >> 4);
    uint32_t rawHum = ((uint32_t) buf[6] << 8) | buf[7];
    uint32_t rawPress = ((uint32_t) buf[0] << 12) | ((uint32_t) buf[1] << 4) | ((uint32_t) buf[2] >> 4);

    int32_t calcTemp = calculateTemperature(rawTemp, calibrationData);

    if (rawTemp != 0x80000) {
        data->temperatureC = (float) calcTemp / 100;
    } else {
        data->temperatureC = BME280_MEASUREMENT_NO_DATA;
    }

    if (rawHum != 0x8000) {
        int32_t calcHum = calculateHumidity(rawHum, calibrationData);
        data->humidityPercent = (float) calcHum / 1024;
    } else {
        data->humidityPercent = BME280_MEASUREMENT_NO_DATA;
    }

    if (rawPress != 0x80000) {
        uint32_t calcPress = calculatePressure(rawPress, calibrationData);
        data->pressurePa = calcPress / 256;
    } else {
        data->pressurePa = BME280_MEASUREMENT_NO_DATA;
    }

    return BME280_OK;
}

float bme280ConvertPressurePaToMmHg(uint32_t pressurePa) {
    return (float) (pressurePa * 0.00750062);
}
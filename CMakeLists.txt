cmake_minimum_required(VERSION 3.9)
project(avr-bme280 C)
set(CMAKE_C_STANDARD 99)

set(MCU "atmega16")
set(F_CPU "8000000")
set(F_I2C "100000")
set(BME280_ADDRESS "0x77")

set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_RANLIB avr-ranlib)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_C_FLAGS "-mmcu=${MCU} -DF_CPU=${F_CPU} -DF_I2C=${F_I2C} -DBME280_ADDRESS=${BME280_ADDRESS}")

include_directories(include)
add_library(${PROJECT_NAME} src/i2c.c src/bme280.c)
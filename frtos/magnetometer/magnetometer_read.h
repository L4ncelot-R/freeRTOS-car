/**
 * @file magnetometer_read.h
 * @author Woon Jun Wei
 * @brief This file contains the functions to read the data
 * from the LSM303DLHC accelerometer and magnetometer sensor
 */

#ifndef MAGNETOMETER_READ_H
#define MAGNETOMETER_READ_H

#include "magnetometer_init.h"

/**
 * @brief Read Data with I2C, given the address and register
 * @param addr  Address of the device
 * @param reg   Register to read from
 * @return      1 piece of data read from the register
 */
static inline int
read_data(uint8_t addr, uint8_t reg) {
    uint8_t data[1];

    // Send the register address to read from
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);

    // Read the data
    i2c_read_blocking(i2c_default, addr, data, 1, false);

    return data[0];
}

/**
 * @brief Read Accelerometer Data
 * @param accelerometer   Accelerometer Data
 */
static inline void
read_accelerometer(int16_t accelerometer[3]) {
    uint8_t buffer[6];

    buffer[0] = read_data(ACCEL_ADDR, LSM303_OUT_X_L_A);
    buffer[1] = read_data(ACCEL_ADDR, LSM303_OUT_X_H_A);
    buffer[2] = read_data(ACCEL_ADDR, LSM303_OUT_Y_L_A);
    buffer[3] = read_data(ACCEL_ADDR, LSM303_OUT_Y_H_A);
    buffer[4] = read_data(ACCEL_ADDR, LSM303_OUT_Z_L_A);
    buffer[5] = read_data(ACCEL_ADDR, LSM303_OUT_Z_H_A);

    // Combine high and low bytes

    // xAcceleration
    accelerometer[0] = (int16_t) ((buffer[1] << 8) | buffer[0]);

    // yAcceleration
    accelerometer[1] = (int16_t) ((buffer[3] << 8) | buffer[2]);

    // zAcceleration
    accelerometer[2] = (int16_t) ((buffer[5] << 8) | buffer[4]);

}

/**
 * @brief Read Magnetometer Data with Moving Average
 * @param magnetometer  Magnetometer Data
 */
static inline void
read_magnetometer(int16_t magnetometer[3]) {
    uint8_t buffer[6];
    int32_t xMagFiltered = 0;
    int32_t yMagFiltered = 0;
    int32_t zMagFiltered = 0;

    for (int i = 0; i < NUM_READINGS; i++) {
        buffer[0] = read_data(MAG_ADDR, LSM303_OUT_X_H_M);
        buffer[1] = read_data(MAG_ADDR, LSM303_OUT_X_L_M);
        buffer[2] = read_data(MAG_ADDR, LSM303_OUT_Y_H_M);
        buffer[3] = read_data(MAG_ADDR, LSM303_OUT_Y_L_M);
        buffer[4] = read_data(MAG_ADDR, LSM303_OUT_Z_H_M);
        buffer[5] = read_data(MAG_ADDR, LSM303_OUT_Z_L_M);

        // Update the cumulative sum of the magnetometer data
        xMagFiltered += (int16_t)(buffer[0] << 8 | buffer[1]);
        yMagFiltered += (int16_t)(buffer[2] << 8 | buffer[3]);
        zMagFiltered += (int16_t)(buffer[4] << 8 | buffer[5]);
    }

    // Calculate the moving average
    magnetometer[0] = xMagFiltered / NUM_READINGS;
    magnetometer[1] = yMagFiltered / NUM_READINGS;
    magnetometer[2] = zMagFiltered / NUM_READINGS;
}

/**
 * @brief Read Temperature Data in Degrees Celsius
 * @param temperature  Temperature Data in Degrees Celsius
 */
static inline void
read_temperature(int16_t temperature[1]) {
    uint8_t buffer[2];

    buffer[0] = read_data(MAG_ADDR, LSM303_TEMP_OUT_H_M);
    buffer[1] = read_data(MAG_ADDR, LSM303_TEMP_OUT_L_M);

    /**
     * Normalize temperature; it is big-endian, fixed-point
     * 9 bits signed integer, 3 bits fractional part, 4 bits zeros
     * and is relative to 20 degrees Celsius
     * Source: https://electronics.stackexchange.com/a/356964
     */

    int16_t raw_temperature =
            (20 << 3) + (((int16_t) buffer[0] << 8 | buffer[1]) >> 4);

    // Convert the raw temperature data to degrees Celsius
    float temperature_celsius = (float) raw_temperature / 8.0;

    // Store the result in the temperature array
    temperature[0] = (int16_t) temperature_celsius;
}

#endif
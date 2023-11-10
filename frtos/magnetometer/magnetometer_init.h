/**
 * @file    magnetometer_init.h
 * @author  Woon Jun Wei
 * @brief   Initialise the magnetometer sensor and
 *          calculate the direction of the car
 *
 * @details This file contains the function prototypes for the
 *          magnetometer sensor and the function to calculate
 *          the direction of the car based on the magnetometer sensor data
 */

#ifndef MAGNETOMETER_INIT_H
#define MAGNETOMETER_INIT_H

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "semphr.h"

#include "car_config.h"
#include "LSM303DLHC_register.h"

// Semaphores
SemaphoreHandle_t g_direction_sem = NULL;

/**
 * @brief Read Data with I2C, given the address and register
 * @param addr  Address of the device
 * @param reg   Register to read from
 * @return      1 piece of data read from the register
 */
static inline int
read_data(uint8_t addr, uint8_t reg)
{
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
read_accelerometer(int16_t accelerometer[3], volatile direction_t *p_direction)
{
    uint8_t buffer[6];

    buffer[0] = read_data(ACCEL_ADDR, LSM303_OUT_X_L_A);
    buffer[1] = read_data(ACCEL_ADDR, LSM303_OUT_X_H_A);
    buffer[2] = read_data(ACCEL_ADDR, LSM303_OUT_Y_L_A);
    buffer[3] = read_data(ACCEL_ADDR, LSM303_OUT_Y_H_A);
    buffer[4] = read_data(ACCEL_ADDR, LSM303_OUT_Z_L_A);
    buffer[5] = read_data(ACCEL_ADDR, LSM303_OUT_Z_H_A);

    // Combine high and low bytes

    // xAcceleration
    accelerometer[0] = (int16_t)((buffer[1] << 8) | buffer[0]);

    // yAcceleration
    accelerometer[1] = (int16_t)((buffer[3] << 8) | buffer[2]);

    // zAcceleration
    accelerometer[2] = (int16_t)((buffer[5] << 8) | buffer[4]);

    // Apply the calibration data
    accelerometer[0] -= p_direction->calibration_data->accelerometerBias[0];
    accelerometer[1] -= p_direction->calibration_data->accelerometerBias[1];
    accelerometer[2] -= p_direction->calibration_data->accelerometerBias[2];
}

/**
 * @brief Read Magnetometer Data with Moving Average
 * @param magnetometer  Magnetometer Data
 */
static inline void
read_magnetometer(int16_t magnetometer[3], volatile direction_t *p_direction)
{
    uint8_t buffer[6];
    int32_t xMagFiltered = 0;
    int32_t yMagFiltered = 0;
    int32_t zMagFiltered = 0;

    for (int i = 0; i < NUM_READINGS; i++)
    {
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

    // Apply the calibration data
    magnetometer[0] -= p_direction->calibration_data->magnetometerBias[0];
    magnetometer[1] -= p_direction->calibration_data->magnetometerBias[1];
    magnetometer[2] -= p_direction->calibration_data->magnetometerBias[2];
}

/**
 * @brief Read Temperature Data in Degrees Celsius
 * @param temperature  Temperature Data in Degrees Celsius
 */
static inline void
read_temperature(int16_t temperature[1])
{
    uint8_t buffer[2];

    buffer[0] = read_data(MAG_ADDR, LSM303_TEMP_OUT_H_M);
    buffer[1] = read_data(MAG_ADDR, LSM303_TEMP_OUT_L_M);

    /**
     * Normalize temperature; it is big-endian, fixed-point
     * 9 bits signed integer, 3 bits fractional part, 4 bits zeros
     * and is relative to 20 degrees Celsius
     * Source: https://electronics.stackexchange.com/a/356964
     */

    int16_t raw_temperature
        = (20 << 3) + (((int16_t)buffer[0] << 8 | buffer[1]) >> 4);

    // Convert the raw temperature data to degrees Celsius
    float temperature_celsius = (float)raw_temperature / 8.0;

    // Store the result in the temperature array
    temperature[0] = (int16_t)temperature_celsius;
}

/**
 * @brief Calibrate the Magnetometer Sensor via bias values
 *
 * @return None
 */
void
initial_calibration(direction_t *p_direction)
{
    int16_t accelerometer[3];
    int16_t magnetometer[3];

    int16_t accelerometerMin[3] = { 0, 0, 0 };
    int16_t accelerometerMax[3] = { 0, 0, 0 };
    int16_t magnetometerMin[3]  = { 0, 0, 0 };
    int16_t magnetometerMax[3]  = { 0, 0, 0 };

    printf("Initial Calibration\n");

    for (int i = 0; i < 100; i++)
    {
        printf("Calibrating... %d\n", i);

        read_accelerometer(accelerometer, p_direction);
        read_magnetometer(magnetometer, p_direction);

        for (int j = 0; j < 3; j++)
        {
            if (accelerometer[j] > accelerometerMax[j])
            {
                accelerometerMax[j] = accelerometer[j];
            }
            if (accelerometer[j] < accelerometerMin[j])
            {
                accelerometerMin[j] = accelerometer[j];
            }
            if (magnetometer[j] > magnetometerMax[j])
            {
                magnetometerMax[j] = magnetometer[j];
            }
            if (magnetometer[j] < magnetometerMin[j])
            {
                magnetometerMin[j] = magnetometer[j];
            }
        }
        sleep_ms(10);
    }

    p_direction->calibration_data->accelerometerBias[0]
        = (accelerometerMax[0] + accelerometerMin[0]) / 2;
    p_direction->calibration_data->accelerometerBias[1]
        = (accelerometerMax[1] + accelerometerMin[1]) / 2;
    p_direction->calibration_data->accelerometerBias[2]
        = (accelerometerMax[2] + accelerometerMin[2]) / 2;

    p_direction->calibration_data->magnetometerBias[0]
        = (magnetometerMax[0] + magnetometerMin[0]) / 2;
    p_direction->calibration_data->magnetometerBias[1]
        = (magnetometerMax[1] + magnetometerMin[1]) / 2;
    p_direction->calibration_data->magnetometerBias[2]
        = (magnetometerMax[2] + magnetometerMin[2]) / 2;

    printf("Accelerometer Bias: %d, %d, %d\n",
           p_direction->calibration_data->accelerometerBias[0],
           p_direction->calibration_data->accelerometerBias[1],
           p_direction->calibration_data->accelerometerBias[2]);

    printf("Magnetometer Bias: %d, %d, %d\n",
           p_direction->calibration_data->magnetometerBias[0],
           p_direction->calibration_data->magnetometerBias[1],
           p_direction->calibration_data->magnetometerBias[2]);
}

/**
 * @brief   Initialise the LSM303DLHC sensor (Accelerometer and Magnetometer)
 * @details
 *          Accelerometer - Normal power mode, all axes enabled, 10 Hz,
 *          Full Scale +-2g, continuous update
 *
 *          Magnetometer - Continuous-conversion mode, Gain = +/- 1.3,
 *          Enable temperature sensor, 220 Hz
 *
 * @return None
 */
static void
LSM303DLHC_init()
{
    /**
     * Accelerometer Setup
     */

    // 0x20 = CTRL_REG1_A
    // Normal power mode, all axes enabled, 10 Hz
    uint8_t buf[2] = { LSM303_CTRL_REG1_A, 0x27 };
    i2c_write_blocking(i2c_default, ACCEL_ADDR, buf, 2, false);

    // Reboot memory content (0x40 = CTRL_REG4_A)
    // Full Scale +-2g, continuous update (0x00 = 0b0000 0000)
    buf[0] = LSM303_CTRL_REG4_A;
    buf[1] = 0x00;
    i2c_write_blocking(i2c_default, ACCEL_ADDR, buf, 2, false);

    /**
     * Magnetometer Setup
     */

    // MR_REG_M (0x02) - Continuous-conversion mode (0x00 -> 00000000)
    buf[0] = LSM303_MR_REG_M;
    buf[1] = 0x00;
    i2c_write_blocking(i2c_default, MAG_ADDR, buf, 2, false);

    // CRB_REG_M (0x01) - Gain = +/- 1.3 (0x20 -> 00100000)
    buf[0] = LSM303_CRB_REG_M;
    buf[1] = 0x20;
    i2c_write_blocking(i2c_default, MAG_ADDR, buf, 2, false);

    // CRA_REG_M (0x00), 0x9C = 0b1001 1100
    // Enable temperature sensor (0x80 -> 1000 0000)
    // 220 Hz (0x1C -> 0001 1100)
    buf[0] = LSM303_CRA_REG_M;
    buf[1] = 0x9C;
    i2c_write_blocking(i2c_default, MAG_ADDR, buf, 2, false);
}

/**
 * @brief Initialise the Magnetometer Sensor
 * @details Initialise the I2C Port, SDA and SCL Pins, and the LSM303DLHC Sensor
 */
void
magnetometer_init(car_struct_t *p_car_struct)
{
    p_car_struct->p_direction->roll        = 0;
    p_car_struct->p_direction->pitch       = 0;
    p_car_struct->p_direction->yaw         = 0;
    p_car_struct->p_direction->orientation = NORTH;
    p_car_struct->p_direction->roll_angle  = LEFT;
    p_car_struct->p_direction->pitch_angle = UP;
    calibration_data_t g_calibration_data
        = { .accelerometerBias = { 0, 0, 0 }, .magnetometerBias = { 0, 0, 0 } };
    p_car_struct->p_direction->calibration_data = &g_calibration_data;

    printf("Magnetometer Initialising\n");
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    LSM303DLHC_init();

    //    initial_calibration();

    printf("Magnetometer Initialised\n");
}

/**
 * @brief Timer Interrupt Handler To calculate the direction of the car
 * @param repeatingTimer The timer handler
 * @return True (To keep the timer running)
 */
bool
h_direction_timer_handler(repeating_timer_t *repeatingTimer)
{

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_direction_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return true;
}

#endif
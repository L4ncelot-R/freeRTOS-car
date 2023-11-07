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

#include "magnetometer_config.h"
#include "LSM303DLHC_register.h"

// Semaphores
SemaphoreHandle_t g_direction_sem = NULL;

direction_t g_direction = {
        .roll = 0,
        .pitch = 0,
        .yaw = 0,
        .orientation = NORTH,
        .roll_angle = LEFT,
        .pitch_angle = UP
};

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
LSM303DLHC_init() {
    /**
     * Accelerometer Setup
     */

    // 0x20 = CTRL_REG1_A
    // Normal power mode, all axes enabled, 10 Hz
    uint8_t buf[2] = {LSM303_CTRL_REG1_A, 0x27};
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
magnetometer_init()
{
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    LSM303DLHC_init();

    // Semaphore
    g_direction_sem = xSemaphoreCreateBinary();
}

/**
 * @brief Timer Interrupt Handler To calculate the direction of the car
 * @param repeatingTimer The timer handler
 * @return True (To keep the timer running)
 */
bool
h_direction_timer_handler(repeating_timer_t *repeatingTimer) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_direction_sem,
                          &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return true;
}
#endif
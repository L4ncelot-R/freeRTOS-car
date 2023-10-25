#include "magnetometer_init.h"

static inline int
read_data(uint8_t addr, uint8_t reg) {
    uint8_t data[1];

    // Send the register address to read from
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);

    // Read the data
    i2c_read_blocking(i2c_default, addr, data, 1, false);

    return data[0];
}

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

static inline void
read_magnetometer(int16_t magnetometer[3]) {
    uint8_t buffer[6];

    buffer[0] = read_data(MAG_ADDR, LSM303_OUT_X_H_M);
    buffer[1] = read_data(MAG_ADDR, LSM303_OUT_X_L_M);
    buffer[2] = read_data(MAG_ADDR, LSM303_OUT_Y_H_M);
    buffer[3] = read_data(MAG_ADDR, LSM303_OUT_Y_L_M);
    buffer[4] = read_data(MAG_ADDR, LSM303_OUT_Z_H_M);
    buffer[5] = read_data(MAG_ADDR, LSM303_OUT_Z_L_M);

    magnetometer[0] = (int16_t) (buffer[0] << 8 | buffer[1]); //xMag

    magnetometer[1] = (int16_t) (buffer[2] << 8 | buffer[3]); //yMag

    magnetometer[2] = (int16_t) (buffer[4] << 8 | buffer[5]); //zMag
}

/**
 * FreeRTOS Tasks
 */


//void
//monitor_magnetometer_task(__unused void *params) {
//    for (;;)
//    {
//        if (xSemaphoreTake(g_magnetometer_sem, portMAX_DELAY) == pdTRUE)
//        {
////            printf("Magnetometer Task");
//            int16_t magnetometer[3];
//            read_magnetometer(magnetometer);
//
//            // Send to message buffer
//            xMessageBufferSend(g_magnetometer_buffer,
//                               &magnetometer,
//                               sizeof(magnetometer),
//                               0
//            );
//
//            printf("Magnetometer: %d, %d, %d\n", magnetometer[0],
//                   magnetometer[1], magnetometer[2]);
//        }
//    }
//}

//void
//monitor_accelerometer_task(__unused void *params) {
//    for (;;)
//    {
//        if (xSemaphoreTake(g_accelerometer_sem, portMAX_DELAY) == pdTRUE)
//        {
//            int16_t accelerometer[3];
//            read_accelerometer(accelerometer);
//
//            // Send to message buffer
//            xMessageBufferSend(g_accelerometer_buffer,
//                               &accelerometer,
//                               sizeof(accelerometer),
//                               0
//            );
//
//            printf("Accelerometer: %d, %d, %d\n", accelerometer[0],
//                   accelerometer[1], accelerometer[2]);
//        }
//    }
//}

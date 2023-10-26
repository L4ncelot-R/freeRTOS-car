#ifndef MAGNETOMETER_REGISTER_H
#define MAGNETOMETER_REGISTER_H

// Accelerometer registers
#define LSM303_CTRL_REG1_A          0x20
#define LSM303_CTRL_REG4_A          0x23
#define LSM303_CTRL_REG5_A          0x24
#define LSM303_OUT_X_L_A            0x28
#define LSM303_OUT_X_H_A            0x29
#define LSM303_OUT_Y_L_A            0x2A
#define LSM303_OUT_Y_H_A            0x2B
#define LSM303_OUT_Z_L_A            0x2C
#define LSM303_OUT_Z_H_A            0x2D

// Magnetometer registers
#define LSM303_CRA_REG_M            0x00
#define LSM303_CRB_REG_M            0x01
#define LSM303_MR_REG_M             0x02
#define LSM303_OUT_X_H_M            0x03
#define LSM303_OUT_X_L_M            0x04
#define LSM303_OUT_Z_H_M            0x05
#define LSM303_OUT_Z_L_M            0x06
#define LSM303_OUT_Y_H_M            0x07
#define LSM303_OUT_Y_L_M            0x08
#define LSM303_SR_REG_M             0x09

// Temperature registers
#define LSM303_TEMP_OUT_H_M         0x31
#define LSM303_TEMP_OUT_L_M         0x32

// LSM303DLHC temperature compensation coefficients
#define SCALE_Z                     0.9 // Scale factor for Z-axis
#define OFFSET_Z                    5.0 // Offset for Z-axis

#define TEMPERATURE_OFFSET          25.0 // Reference temperature for calibration
#define TEMPERATURE_COEFFICIENT_Z   0.33

#define OFFSET_Z                    5.0
#define SCALE_Z                     0.9

#define ACCEL_ADDR                  0x19
#define MAG_ADDR                    0x1E

#endif
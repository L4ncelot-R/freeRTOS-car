#ifndef MAGNETOMETER_CONFIG_H
#define MAGNETOMETER_CONFIG_H

#define I2C_PORT                                i2c0
#define I2C_SDA                                 ( 8 )
#define I2C_SCL                                 ( 9 )

#define DIRECTION_READ_DELAY                    ( 100 )

#define NUM_READINGS                            ( 10 ) // Number of readings to
                                                        // take before
                                                        // calculating
                                                        // direction

//#define ALPHA                                   ( 0.1f ) // Low Pass Filter
                                                          // Coefficient

// LSM303DLHC temperature compensation coefficients
#define SCALE_Z                                 ( 1.0f ) // Scale for Z-axis
#define OFFSET_Z                                ( 0.0f ) // Offset for Z-axis

#define TEMPERATURE_OFFSET                      ( 32.0f ) // Reference
                                                          // temperature for
                                                          // calibration

#define TEMPERATURE_COEFFICIENT_Z               ( 0.33f ) // Temperature
                                                          // coefficient for
                                                          // Z-axis

/**
 * @brief The orientation of the car
 */

typedef enum {
    NORTH,
    NORTH_EAST,
    EAST,
    SOUTH_EAST,
    SOUTH,
    SOUTH_WEST,
    WEST,
    NORTH_WEST
} compass_direction_t;

/**
 * Angle of the car
 */
typedef enum {
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3
} angle_t;

/**
 * @brief The direction of the car
 * roll = angle of the car (left or right)
 * pitch = angle of the car (up or down)
 * heading = direction of the car (north, east, south, west) in degrees
 * orientation = orientation of the car (north, east, south, west)
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    compass_direction_t orientation;
    angle_t roll_angle;
    angle_t pitch_angle;
} direction_t;

#endif

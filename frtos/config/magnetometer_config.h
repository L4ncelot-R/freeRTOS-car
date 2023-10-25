#ifndef MAGNETOMETER_CONFIG_H
#define MAGNETOMETER_CONFIG_H

#define I2C_PORT                                i2c0
#define I2C_SDA                                 ( 8 )
#define I2C_SCL                                 ( 9 )

#define DIRECTION_READ_DELAY                    ( 100 )

#define ALPHA                                   ( 0.98f ) // Complementary
                                                          // Filter Constant

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

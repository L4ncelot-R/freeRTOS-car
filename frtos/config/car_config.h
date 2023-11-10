#include "motor_config.h"
#include "ultrasonic_sensor_config.h"
#include "magnetometer_config.h"
#include "line_sensor_config.h"

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#define PRIO (tskIDLE_PRIORITY + 1UL)

typedef struct s_obs_struct
{
    bool line_detected;
    bool ultrasonic_detected;

} obs_t;

// Define the Map structure
typedef struct {
    bool **data;  // 2D array to represent the grid
    int rows;     // Number of rows in the grid
    int cols;     // Number of columns in the grid
    int initial_x;
    int initial_y;
    float * distance_array;
    int distance_array_size;
} grid_t;

typedef struct
{
    obs_t       *obs;
    motor_t     *p_left_motor;
    motor_t     *p_right_motor;
    motor_pid_t *p_pid;
    direction_t *p_direction;
    grid_t      *p_grid;

} car_struct_t;

#endif // CAR_CONFIG_H

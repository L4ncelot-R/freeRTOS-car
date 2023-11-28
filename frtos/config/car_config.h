#include "motor_config.h"
#include "ultrasonic_sensor_config.h"
#include "magnetometer_config.h"
#include "line_sensor_config.h"

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#define PRIO       (tskIDLE_PRIORITY + 1UL)
#define MAX_HEIGHT 10
#define MAX_WIDTH  10

typedef struct s_obs_struct
{
    bool left_sensor_detected;
    bool right_sensor_detected;
    bool ultrasonic_detected;

} obs_t;

typedef enum e_direction
{
    up,
    down,
    left,
    right
} mapping_direction_t;

typedef struct
{
    char type;
    int  reachable;
    int  visited;
} mazecells_t;

typedef struct
{
    int         height;
    int         width;
    mazecells_t mazecells[MAX_HEIGHT][MAX_WIDTH];
} maze_t;

typedef struct
{
    obs_t       *obs;
    motor_t     *p_left_motor;
    motor_t     *p_right_motor;
    motor_pid_t *p_pid;
    direction_t *p_direction;
    maze_t      *p_maze;

} car_struct_t;

#endif // CAR_CONFIG_H

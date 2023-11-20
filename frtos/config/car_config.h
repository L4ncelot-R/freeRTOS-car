#include "motor_config.h"
#include "ultrasonic_sensor_config.h"
#include "magnetometer_config.h"
#include "line_sensor_config.h"

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#define PRIO (tskIDLE_PRIORITY + 1UL)

typedef struct s_obs_struct
{
    bool left_sensor_detected;
    bool right_sensor_detected;
    bool ultrasonic_detected;

} obs_t;

typedef struct
{
    obs_t       *obs;
    motor_t     *p_left_motor;
    motor_t     *p_right_motor;
    motor_pid_t *p_pid;
    direction_t *p_direction;

} car_struct_t;

#endif // CAR_CONFIG_H

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/adc.h"
#include "Config.h"

typedef enum
{
    LINE_DETECTED = 1,
    LINE_NOT_DETECTED = 0,
} state_t;

typedef enum
{
    ERROR = 0,
    RIGHT = 1,
    LEFT = 2,
    FORWARD = 3
} direction_t;

typedef enum
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
} orientation_t;

typedef struct
{
    u_int8_t x;                     // Current x coordinate
    u_int8_t y;                     // Current y coordinate
    direction_t current_direction;  // Current direction (forward, left, right)
    orientation_t orientation;      // Current orientation (N, E, S, W)
} car_state_t;

void line_sensor_setup();

static inline state_t get_state(uint gpio);
static inline direction_t get_current_dir();

void line_sensor_task(__unused void *params);

#endif
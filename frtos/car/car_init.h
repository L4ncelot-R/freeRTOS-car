/**
 * @brief Car Init
 */

#ifndef CAR_INIT_H
#define CAR_INIT_H

#include <stdio.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "line_sensor_config.h"

typedef enum {
    ERROR = 0,
    RIGHT = 1,
    LEFT = 2,
    FORWARD = 3
} direction_t;

typedef enum {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
} orientation_t;

typedef struct {
    u_int8_t x;                     // Current x coordinate
    u_int8_t y;                     // Current y coordinate
    direction_t current_direction;  // Current direction (forward, left, right)
    orientation_t orientation;      // Current orientation (N, E, S, W)
} car_state_t;

/* Common Car State Structure (TODO: TBC)*/

static car_state_t g_car_state;



#endif /* CAR_INIT_H */
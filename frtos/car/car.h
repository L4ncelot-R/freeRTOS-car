/**
 * @brief Common Car Functionalities for the RTOS Car
 */

#ifndef CAR_H
#define CAR_H

#include "car_init.h"

static car_state_t initialize_car_state() {
    g_car_state.x = MAP_SIZE >> 1;
    g_car_state.y = MAP_SIZE >> 1;
    g_car_state.current_direction = FORWARD;
    g_car_state.orientation = NORTH;

    return g_car_state;
}


#endif /* CAR_H */
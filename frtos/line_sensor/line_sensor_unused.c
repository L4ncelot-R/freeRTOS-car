#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "line_sensor.h"
#include "string.h"

//const float conversionFactor = 3.3f / (1 << 12);

volatile u_int8_t map[MAP_SIZE][MAP_SIZE] = {0};


/**
 * @brief Update the map based on the car's state
 *
 * @param car_state The current car state
 */
static inline void
update_map(car_state_t car_state) {
    if (car_state.x >= 0 && car_state.x < MAP_SIZE &&
        car_state.y >= 0 && car_state.y < MAP_SIZE) {
        map[car_state.x][car_state.y] = 1;
    }
}

/**
 * @brief Handle forward movement of the car
 *
 * @param car_state The current car state
 */
static void
handle_forward_movement(car_state_t *car_state) {
    printf("FORWARD, ");
    // TODO: Check car's actual forward movement
    switch (car_state->orientation) {
        case NORTH:
            printf("NORTH\n");
            car_state->y++;
            break;
        case EAST:
            printf("EAST\n");
            car_state->x++;
            break;
        case SOUTH:
            printf("SOUTH\n");
            car_state->y--;
            break;
        case WEST:
            printf("WEST\n");
            car_state->x--;
            break;
    }
}

/**
 * @brief Handle a right turn of the car
 *
 * Note: Bitwise AND with 0x03 to ensure that the orientation
 * is always between 0 and 3
 * @param car_state The current car state
 */
static inline void
handle_right_turn(car_state_t *car_state) {
    car_state->orientation = (car_state->orientation + 1) & 0x03;
}

/**
 * @brief Handle a left turn of the car
 *
 * @param car_state The current car state
 */
static inline void
handle_left_turn(car_state_t *car_state) {
    car_state->orientation = (car_state->orientation - 1) & 0x03;
}

/**
 * @brief Print the map to the console
 *
 * This function will print the map to the console
 */
void
print_map() {
    // Invert the map, 0,0 is at the bottom left
    for (int i = MAP_SIZE - 1; i >= 0; i --)
    {
        for (int j = 0; j < MAP_SIZE; j ++)
        {
            printf("%d ", map[j][i]);
        }
        printf("\n");
    }
}


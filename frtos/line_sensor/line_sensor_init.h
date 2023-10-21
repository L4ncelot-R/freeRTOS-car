/**
 * @file line_sensor_init.h
 * @brief Initialise the line sensor
 * @author Woon Jun Wei
 */

#ifndef LINE_SENSOR_INIT_H
#define LINE_SENSOR_INIT_H

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "semphr.h"

#include "line_sensor_config.h"

typedef enum { // Unused, useful for readability
    LINE_DETECTED = 0,
    LINE_NOT_DETECTED = 1,
} state_t;

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

// Semaphore
SemaphoreHandle_t g_left_sensor_sem = NULL;
SemaphoreHandle_t g_right_sensor_sem = NULL;

// Queue
static MessageBufferHandle_t left_sensor_msg_buffer;   // Left Sensor Buffer
static MessageBufferHandle_t right_sensor_msg_buffer;   // Right Sensor Buffer

// Car State Struct
static car_state_t g_car_state;

static car_state_t initialize_car_state() {
    g_car_state.x = MAP_SIZE >> 1;
    g_car_state.y = MAP_SIZE >> 1;
    g_car_state.current_direction = FORWARD;
    g_car_state.orientation = NORTH;

    return g_car_state;
}

/**
 * @brief Setup the Line Sensor
 *
 * This function will setup the Line Sensor by initializing it as an input
 */
static inline void
line_sensor_setup() {
    g_left_sensor_sem       = xSemaphoreCreateBinary();
    g_right_sensor_sem      = xSemaphoreCreateBinary();

    uint mask               = (1 << LEFT_SENSOR_PIN) | (1 << RIGHT_SENSOR_PIN);

    // Initialise 2 GPIO pins and set them to input
    gpio_init_mask(mask);
    gpio_set_dir_in_masked(mask);

    left_sensor_msg_buffer  = xMessageBufferCreate(30);
    right_sensor_msg_buffer = xMessageBufferCreate(30);

}

/**
 * @brief Timer Interrupt Handler for the left sensor
 * @param rt
 * @return True (To keep the timer running)
 */
bool h_left_sensor_timer_handler(repeating_timer_t *repeatingTimer) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_left_sensor_sem,
                          &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return true;
}

/**
 * @brief Timer Interrupt Handler for the right sensor
 *
 * @param repeatingTimer
 * @return True (To keep the timer running)
 */
bool h_right_sensor_timer_handler(repeating_timer_t *repeatingTimer) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_right_sensor_sem,
                          &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return true;
}
#endif /* LINE_SENSOR_INIT_H */

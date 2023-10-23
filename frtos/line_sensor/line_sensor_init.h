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

#define DEBOUNCE_DELAY_MS 100

static TickType_t lastEdgeTimeLeft = 0;
static TickType_t lastEdgeTimeRight = 0;

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

static volatile BaseType_t right_sensor_triggered = pdFALSE;
static volatile BaseType_t left_sensor_triggered = pdFALSE;

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
    g_left_sensor_sem = xSemaphoreCreateBinary();
    g_right_sensor_sem = xSemaphoreCreateBinary();

    uint mask = (1 << LEFT_SENSOR_PIN) | (1 << RIGHT_SENSOR_PIN);

    // Initialise 2 GPIO pins and set them to input
    gpio_init_mask(mask);
    gpio_set_dir_in_masked(mask);

    left_sensor_msg_buffer = xMessageBufferCreate(30);
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

void h_line_sensor_handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TickType_t currentTicks = xTaskGetTickCount();
    printf("Interrupt triggered\n");

    if (gpio_get_irq_event_mask(LEFT_SENSOR_PIN) & GPIO_IRQ_EDGE_FALL)
    {
        if ((currentTicks - lastEdgeTimeLeft) >=
            pdMS_TO_TICKS(DEBOUNCE_DELAY_MS))
        {
            lastEdgeTimeLeft = currentTicks;
            gpio_acknowledge_irq(LEFT_SENSOR_PIN, GPIO_IRQ_EDGE_FALL);

            left_sensor_triggered = pdTRUE;
            xSemaphoreGiveFromISR(g_left_sensor_sem, &xHigherPriorityTaskWoken);
        }
        else
        {
            // Reset the timer to the currentTicks if the edge is ignored
            lastEdgeTimeLeft = currentTicks;
        }
    }

    if (gpio_get_irq_event_mask(RIGHT_SENSOR_PIN) & GPIO_IRQ_EDGE_FALL)
    {
        if ((currentTicks - lastEdgeTimeRight) >=
            pdMS_TO_TICKS(DEBOUNCE_DELAY_MS))
        {
            lastEdgeTimeRight = currentTicks;
            gpio_acknowledge_irq(RIGHT_SENSOR_PIN, GPIO_IRQ_EDGE_FALL);
            // Set the flag to notify the task
            right_sensor_triggered = pdTRUE;
            xSemaphoreGiveFromISR(g_right_sensor_sem,
                                  &xHigherPriorityTaskWoken);
        }
        else
        {
            // Reset the timer to the currentTicks if the edge is ignored
            lastEdgeTimeRight = currentTicks;
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#endif /* LINE_SENSOR_INIT_H */

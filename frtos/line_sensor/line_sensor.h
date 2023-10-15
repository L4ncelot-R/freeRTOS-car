#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "semphr.h"

#include "Config.h"

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
    g_left_sensor_sem   = xSemaphoreCreateBinary();
    g_right_sensor_sem  = xSemaphoreCreateBinary();

    // Setup GPIO Interrupts
    uint mask           = (1 << LEFT_SENSOR_PIN) | (1 << RIGHT_SENSOR_PIN);

    // Initialise 2 GPIO pins and set them to input
    gpio_init_mask(mask);
    gpio_set_dir_in_masked(mask);

    left_sensor_msg_buffer  = xMessageBufferCreate(30);
    right_sensor_msg_buffer = xMessageBufferCreate(30);

}


/**
 * @brief Monitor the left sensor
 *
 * This function will monitor the left sensor and send the state to the
 * left sensor message buffer, used to calculate the direction of the car
 *
 * @param params
 */
void
monitor_left_sensor_task(__unused void *params) {
    for (;;)
    {
        if (xSemaphoreTake(g_left_sensor_sem, portMAX_DELAY) == pdTRUE)
        {
            // Get Current State
            state_t state = gpio_get(LEFT_SENSOR_PIN);

            xMessageBufferSend(left_sensor_msg_buffer,
                               &state,
                               sizeof(state_t),
                               0);

        }
    }
}

/**
 * @brief Monitor the right sensor
 *
 * This function will monitor the right sensor and send the state to the
 * right sensor message buffer, used to calculate the direction of the car
 *
 * @param params
 */
void
monitor_right_sensor_task(__unused void *params) {
    for (;;)
    {
        if (xSemaphoreTake(g_right_sensor_sem, portMAX_DELAY) == pdTRUE)
        {
            // Get Current State
            state_t state = gpio_get(RIGHT_SENSOR_PIN);

            xMessageBufferSend(right_sensor_msg_buffer,
                               &state,
                               sizeof(state_t),
                               0);
        }
    }
}

/**
 * @brief Monitor the direction and Oritentation of the car
 *
 * This function will monitor the direction and orientation of the car
 * and update the car state accordingly
 *
 * @param params
 */
void
monitor_direction_task(__unused void *params) {
    state_t left_state;
    state_t right_state;

    for (;;)
    {
        // Receive from Buffer
        xMessageBufferReceive(left_sensor_msg_buffer,
                              &left_state,
                              sizeof(state_t),
                              portMAX_DELAY);

        xMessageBufferReceive(right_sensor_msg_buffer,
                              &right_state,
                              sizeof(state_t),
                              portMAX_DELAY);

        g_car_state.current_direction = (left_state << 1) | right_state;

        switch (g_car_state.current_direction)
        {
            case FORWARD:
                break;
            case RIGHT:
                g_car_state.orientation = (g_car_state.orientation + 1) & 0x03;
                break;
            case LEFT:
                g_car_state.orientation = (g_car_state.orientation - 1) & 0x03;
                break;
            default:
                break;
        }

        switch (g_car_state.current_direction)
        {
            case FORWARD:
                printf("Direction: Forward\n");
                break;
            case RIGHT:
                printf("Direction: Right\n");
                break;
            case LEFT:
                printf("Direction: Left\n");
                break;
            default:
                printf("Direction: Error\n");
                break;
        }

        switch (g_car_state.orientation)
        {
            case NORTH:
                printf("Orientation: North\n");
                break;
            case EAST:
                printf("Orientation: East\n");
                break;
            case SOUTH:
                printf("Orientation: South\n");
                break;
            case WEST:
                printf("Orientation: West\n");
                break;
            default:
                printf("Orientation: Error\n");
                break;
        }
    }
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

/**
* Main Launch body
 * This is to be in the main launch function for FreeRTOS
 *
 * // Repeating timer
 *   struct repeating_timer g_left_sensor_timer;
 *   add_repeating_timer_ms(100,
 *                          h_left_sensor_timer_handler,
 *                          NULL,
 *                          &g_left_sensor_timer);

 *   struct repeating_timer g_right_sensor_timer;
 *   add_repeating_timer_ms(100,
 *                          h_right_sensor_timer_handler,
 *                          NULL,
 *                          &g_right_sensor_timer);


 *   TaskHandle_t h_monitor_left_sensor_task;
 *   xTaskCreate(monitor_left_sensor_task,
 *               "Monitor Left Sensor Task",
 *               configMINIMAL_STACK_SIZE,
 *               NULL,
 *               LEFT_TASK_PRIORITY,
 *               &h_monitor_left_sensor_task);

 *   TaskHandle_t h_monitor_right_sensor_task;
 *   xTaskCreate(monitor_right_sensor_task,
 *               "Monitor Right Sensor Task",
 *               configMINIMAL_STACK_SIZE,
 *               NULL,
 *               RIGHT_TASK_PRIORITY,
 *               &h_monitor_right_sensor_task);

 *   TaskHandle_t h_monitor_direction_task;
 *   xTaskCreate(monitor_direction_task,
 *               "Monitor Direction Task",
 *               configMINIMAL_STACK_SIZE,
 *               NULL,
 *               DIRECTION_TASK_PRIORITY,
 *               &h_monitor_direction_task);
*/
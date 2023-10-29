/**
 * @file line_sensor.h
 * @brief Monitor the line sensor and update the car state accordingly
 * @author Woon Jun Wei
 */

#include "line_sensor_init.h"


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
            if (left_sensor_triggered == pdTRUE)
            {
                printf("left sensor triggered\n");
                // Get Current State
                state_t state = gpio_get(LEFT_SENSOR_PIN);

                xMessageBufferSend(left_sensor_msg_buffer,
                                   &state,
                                   sizeof(state_t),
                                   0);
                // Reset the flag
                left_sensor_triggered = pdFALSE;
            }
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
monitor_right_sensor_task(void *params) {
    for (;;) {
        if (xSemaphoreTake(g_right_sensor_sem, portMAX_DELAY) == pdTRUE) {
            // Check the flag or receive the message
            if (right_sensor_triggered == pdTRUE) {
                printf("right sensor triggered\n");
                // Get Current State
                state_t state = gpio_get(RIGHT_SENSOR_PIN);

                xMessageBufferSend(right_sensor_msg_buffer,
                                   &state,
                                   sizeof(state_t),
                                   0);
                // Reset the flag
                right_sensor_triggered = pdFALSE;
            }
        }
    }
}

/**
 * @brief Monitor the barcode sensor
 *
 * This function will monitor the barcode sensor and send the state to the
 * barcode sensor message buffer, used to scan the barcode below the car
 *
 * @param params
 */
void monitor_barcode_sensor_task(void *params) {
    for (;;) {
        if (xSemaphoreTake(g_barcode_sensor_sem, portMAX_DELAY) == pdTRUE) {
            // Check the flag or receive the message
            if (barcode_sensor_triggered == pdTRUE) {
                uint32_t barcode_data = 0;

                for (int i = 0; i < 9; i++) {
                    sleep_ms(100);  // Wait for a segment of the barcode

                    if (gpio_get(BARCODE_SENSOR_PIN)) {
                        barcode_data |= (1u << i);
                    } else {
                        barcode_data &= ~(1u << i);
                    }
                }

                printf("Barcode Data (binary): %09b\n", barcode_data);

                // Send or process the barcode data
                xMessageBufferSend(barcode_sensor_msg_buffer, &barcode_data, sizeof(uint32_t), 0);

                // Reset the flag
                barcode_sensor_triggered = pdFALSE;
            }
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
    state_t barcode_state;

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
        
        xMessageBufferReceive(barcode_sensor_msg_buffer,
                              &barcode_state,
                              sizeof(state_t),
                              portMAX_DELAY);

//        g_car_state.current_direction = (left_state << 1) | right_state;

//        switch (g_car_state.current_direction)
//        {
//            case FORWARD:
//                break;
//            case RIGHT:
//                g_car_state.orientation = (g_car_state.orientation + 1) & 0x03;
//                break;
//            case LEFT:
//                g_car_state.orientation = (g_car_state.orientation - 1) & 0x03;
//                break;
//            default:
//                break;
//        }
//
//        switch (g_car_state.current_direction)
//        {
//            case FORWARD:
//                printf("Direction: Forward\n");
//                break;
//            case RIGHT:
//                printf("Direction: Right\n");
//                break;
//            case LEFT:
//                printf("Direction: Left\n");
//                break;
//            default:
//                printf("Direction: Error\n");
//                break;
//        }
//
//        switch (g_car_state.orientation)
//        {
//            case NORTH:
//                printf("Orientation: North\n");
//                break;
//            case EAST:
//                printf("Orientation: East\n");
//                break;
//            case SOUTH:
//                printf("Orientation: South\n");
//                break;
//            case WEST:
//                printf("Orientation: West\n");
//                break;
//            default:
//                printf("Orientation: Error\n");
//                break;
//        }
    }
}
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
#include "car_config.h"

// Semaphore
SemaphoreHandle_t g_left_sensor_sem = NULL;

/**
 * @brief Setup the Line Sensor
 *
 * This function will setup the Line Sensor by initializing it as an input
 */
static inline void
line_sensor_init() {
//    obs_t obs = {0, 0};
//
//    p_car->obs = &obs;

    g_left_sensor_sem = xSemaphoreCreateBinary();

    uint mask = (1 << LEFT_SENSOR_PIN) | (1 << RIGHT_SENSOR_PIN);

    // Initialise 3 GPIO pins and set them to input
    gpio_init_mask(mask);
    gpio_set_dir_in_masked(mask);

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


void
monitor_left_sensor_task(void *pvParameters) {
    volatile obs_t *p_obs = NULL;
    p_obs = (obs_t *) pvParameters;

    for (;;)
    {
//        if (xSemaphoreTake(g_left_sensor_sem, portMAX_DELAY) == pdTRUE)
//        {
            // Set the flag to notify the task
            p_obs->line_detected = gpio_get(LEFT_SENSOR_PIN);
            printf("Left Sensor: %d\n", p_obs->line_detected);
            vTaskDelay(pdMS_TO_TICKS(LINE_SENSOR_READ_DELAY));
//        }
    }
}


#endif /* LINE_SENSOR_INIT_H */

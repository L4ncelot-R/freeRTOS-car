/**
 * @file ultrasonic_init.h
 * @brief define the constants and initialize the ultrasonic sensor
 * @author Poon Xiang Yuan
 */

#ifndef ULTRASONIC_INIT_H
#define ULTRASONIC_INIT_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "car_config.h"
#include "ultrasonic_sensor.h"

void
check_obstacle(void *pvParameters)
{
    while (true)
    { // Put trigger pin high for 10us
        gpio_put(TRIG_PIN, 1);
        vTaskDelay(1);
        gpio_put(TRIG_PIN, 0);

        // Wait for echo pin to go high
        while (gpio_get(ECHO_PIN) == 0)
            tight_loop_contents();

        // Measure the pulse width (time taken for the echo to return)
        uint32_t start_time = time_us_32();
        while (gpio_get(ECHO_PIN) == 1)
            tight_loop_contents();

        uint32_t end_time = time_us_32();

        // Calculate the distance (in centimeters)
        uint32_t pulse_duration = end_time - start_time;
        float    distance
            = (pulse_duration * 0.034 / 2); // Speed of sound in air to cm/us

        // printf("Distance: %.2f cm\n", distance);

        // change value of obstacle_detected in ultrasonic_t struct
        obs_t *ultrasonic_sensor               = (obs_t *)pvParameters;
        ultrasonic_sensor->ultrasonic_detected = (distance < 7);

        printf("Distance: %.2f cm, Obstacle Detected: %d\n",
               distance,
               ultrasonic_sensor->ultrasonic_detected);
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SENSOR_READ_DELAY));
    }
}


/**
 * @brief Initialise the Ultrasonic Sensor
 * @details Initialise the Ultrasonic Sensor Pins and set default collision value
 */
void
ultrasonic_init(car_struct_t *car_struct)
{
    car_struct->obs->ultrasonic_detected = false;

    // Set up the echo pin
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    // Set up the trigger pin
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
}

void
ultrasonic_task_init(car_struct_t *car_struct)
{
    TaskHandle_t h_monitor_ultrasonic_task = NULL;
    xTaskCreate(check_obstacle,
                "read_ultrasonic_task",
                configMINIMAL_STACK_SIZE,
                (void *)car_struct->obs,
                PRIO,
                &h_monitor_ultrasonic_task);
}

#endif /* ULTRASONIC_INIT_H */
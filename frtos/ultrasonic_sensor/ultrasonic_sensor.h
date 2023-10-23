/**
 * @file ultrasonic_sensor.h
 * @brief Monitor the distance between the car and the wall
 * @author Poon Xiang Yuan
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "ultrasonic_init.h"

void distance_task(__unused void *params)
{
    while (true)
    {
        vTaskDelay(500);

        // Trigger the ultrasonic sensor
        gpio_put(TRIG_PIN, 1);
        sleep_us(10); // Keep the trigger on for 10 microseconds
        gpio_put(TRIG_PIN, 0);

        // Wait for the echo pulse to start
        while (gpio_get(ECHO_PIN) == 0)
            tight_loop_contents();

        // Measure the pulse width (time taken for the echo to return)
        uint32_t start_time = time_us_32();
        while (gpio_get(ECHO_PIN) == 1)
            tight_loop_contents();
        uint32_t end_time = time_us_32();

        // Calculate the distance (in centimeters)
        uint32_t pulse_duration = end_time - start_time;
        float distance = pulse_duration * 0.017; // Speed of sound at ~343 m/s

        printf("Distance: %.2f cm\n", distance);

        // If gonna bang wall
        //
        if (distance < 5)
        {
            printf("Collision Imminent!\n");
            // Proc stop
            //
        }
    }
}
#endif /* ULTRASONIC_SENSOR_H */

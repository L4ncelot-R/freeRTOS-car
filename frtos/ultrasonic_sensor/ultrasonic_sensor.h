/**
 * @file ultrasonic_sensor.h
 * @brief Monitor the distance between the car and the wall
 * @author Poon Xiang Yuan
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "ultrasonic_init.h"

volatile uint32_t start_time;
volatile uint32_t end_time;
volatile bool echo_rising = false;

float
KalmanFilter(float U)
{
    static float R = 10;    // noise convariance can be 10, higher better smooth
    static float H = 1;     // Measurement Map scalar
    static float Q = 10;    // initial estimated convariance
    static float P = 0;     // initial error covariance
    static float U_hat = 0; // initial estimated state
    static float K = 0;     // initial Kalman gain

    // Predict
    //
    K = P * H / (H * P * H + R);            // Update Kalman gain
    U_hat = U_hat + K * (U - H * U_hat);    // Update estimated state

    // Update error covariance
    //
    P = (1 - K * H) * P + Q;

    return U_hat;
}

void 
echo_handler()
{
    if (gpio_get(ECHO_PIN))
    {
        start_time = time_us_32();
        echo_rising = true;
    }
    else
    {
        end_time = time_us_32();
        echo_rising = false;
    }
}

void
distance_task(__unused void *params)
{
    while (true)
    {
        vTaskDelay(1000);

        gpio_put(TRIG_PIN, 1);
        sleep_us(10);
        gpio_put(TRIG_PIN, 0);

        while (echo_rising)
        {
            tight_loop_contents();
        }

        // Calculate the distance (in centimeters)
        uint32_t pulse_duration = end_time - start_time;
        float distance = (pulse_duration * 0.034 / 2) * 1.125; // Speed of sound in cm/us

        // printf("Distance: %.2f cm\n", distance);
        printf("Kalman Filtered Distance: %.2f cm\n", KalmanFilter(distance));

        if (distance < 5)
        {
            printf("Collision Imminent!\n");
        }
    }
}
#endif /* ULTRASONIC_SENSOR_H */

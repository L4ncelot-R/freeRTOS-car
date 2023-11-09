/**
 * @file ultrasonic_sensor.h
 * @brief Monitor the distance between the car and the wall
 * @author Poon Xiang Yuan
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "ultrasonic_init.h"
#include "motor_speed.h"

// volatile uint32_t start_time;
// volatile uint32_t end_time;
// volatile bool     echo_rising = false;

float
KalmanFilter(float U)
{
    static float R = 10;    // noise convariance can be 10, higher better smooth
    static float H = 1;     // Measurement Map scalar
    static float Q = 10;    // initial estimated convariance
    static float P = 0;     // initial error covariance
    static float U_hat = 0; // initial estimated state
    static float K     = 0; // initial Kalman gain

    // Predict
    //
    K     = P * H / (H * P * H + R);     // Update Kalman gain
    U_hat = U_hat + K * (U - H * U_hat); // Update estimated state

    // Update error covariance
    //
    P = (1 - K * H) * P + Q;

    return U_hat;
}

// void
// echo_handler()
// {
//     if (gpio_get(ECHO_PIN))
//     {
//         start_time  = time_us_32();
//         echo_rising = true;
//     }
//     else
//     {
//         end_time    = time_us_32();
//         echo_rising = false;
//     }
// }

void
check_obstacle(void *pvParameters)
{
    while (true)
    { // Put trigger pin high for 10us
        gpio_put(TRIG_PIN, 1);
        sleep_us(10);
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
            = (pulse_duration * 0.034 / 2); // Speed of sound in cm/us

        // printf("Distance: %.2f cm\n", distance);

        // change value of obstacle_detected in ultrasonic_t struct
        ultrasonic_t *ultrasonic_sensor      = (ultrasonic_t *)pvParameters;
        ultrasonic_sensor->obstacle_detected = (distance < 7);

        printf("Distance: %.2f cm, Obstacle Detected: %d\n",
               distance,
               ultrasonic_sensor->obstacle_detected);
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SENSOR_READ_DELAY));
    }
}

void
check_global(void *pvParameters)
{
    while (true)
    {
        ultrasonic_t *ultrasonic_sensor      = (ultrasonic_t *)pvParameters;

        printf("Global Obstacle Detected : %d\n",
               ultrasonic_sensor->obstacle_detected);
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SENSOR_READ_DELAY));
    }
}

// void
// distance_task(__unused void *params)
// {
//     while (true)
//     {
//         vTaskDelay(1000);

//         gpio_put(TRIG_PIN, 1);
//         sleep_us(10);
//         gpio_put(TRIG_PIN, 0);

//         while (gpio_get(ECHO_PIN) == 0)
//             tight_loop_contents();

//         // Measure the pulse width (time taken for the echo to return)
//         uint32_t start_time = time_us_32();
//         while (gpio_get(ECHO_PIN) == 1)
//             tight_loop_contents();
//         uint32_t end_time = time_us_32();

//         // Calculate the distance (in centimeters)
//         uint32_t pulse_duration = end_time - start_time;
//         float    distance
//             = (pulse_duration * 0.034 / 2); // Speed of sound in cm/us

//         printf("Distance: %.2f cm\n", distance);
//         // printf("Kalman Filtered Distance: %.2f cm\n",
//         // KalmanFilter(distance));

//         if (distance < 7)
//         {
//             // set_wheel_direction(DIRECTION_MASK);
//             set_wheel_speed_synced(0u);
//             printf("Collision Imminent!\n");
//             vTaskDelay(pdMS_TO_TICKS(3000));
//             spin_to_yaw(350);
//             set_wheel_direction(DIRECTION_FORWARD);
//             set_wheel_speed_synced(90u);
//         }
//         start_time, end_time = 0;
//     }
// }
#endif /* ULTRASONIC_SENSOR_H */
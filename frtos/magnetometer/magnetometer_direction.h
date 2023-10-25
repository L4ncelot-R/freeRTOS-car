#include "magnetometer_init.h"

static char
*direction(int16_t acceleration[3], int16_t magnetometer[3]) {
    // Calculate the angle from accelerometer data
    float roll = atan2(acceleration[1], acceleration[2]) * (180.0 / M_PI);
    float pitch = atan2(- acceleration[0],
                        sqrt(acceleration[1] * acceleration[1] +
                             acceleration[2] * acceleration[2])) *
                  (180.0 / M_PI);

    // Calculate the heading from magnetometer data
    float heading = atan2(magnetometer[1], magnetometer[0]) * (180.0 / M_PI);

    // Adjust the heading for negative values
    if (heading < 0)
    {
        heading += 360.0;
    }

    // Determine the direction based on the heading
    // TODO: Optimize this
    char *dir;

    if (heading >= 315 || heading < 45)
    {
        dir = "North";
    }
    else
    {
        if (heading >= 45 && heading < 135)
        {
            dir = "East";
        }
        else
        {
            if (heading >= 135 && heading < 225)
            {
                dir = "South";
            }
            else
            {
                dir = "West";
            }
        }
    }

    printf("Roll: %f, Pitch: %f, Heading: %f\n", roll, pitch, heading);

    return dir;
}

/**
 * FreeRTOS Tasks
 */

void
monitor_direction_task(__unused void *params) {
    for (;;)
    {
        if (xSemaphoreTake(g_direction_sem, portMAX_DELAY) == pdTRUE)
        {
            // Read from message buffer
            int16_t magnetometer[3];
            int16_t accelerometer[3];

            read_magnetometer(magnetometer);
            read_accelerometer(accelerometer);

            // Calculate the angle from accelerometer data
            float roll =
                    atan2(accelerometer[1], accelerometer[2]) * (180.0 / M_PI);
            float pitch = atan2(- accelerometer[0],
                                sqrt(accelerometer[1] * accelerometer[1] +
                                     accelerometer[2] * accelerometer[2])) *
                          (180.0 / M_PI);

            // Calculate the heading from magnetometer data
            float heading =
                    atan2(magnetometer[1], magnetometer[0]) * (180.0 / M_PI);

            // Adjust the heading for negative values
            if (heading < 0)
            {
                heading += 360.0;
            }

            // Determine the direction based on the heading
            char *dir;

            if (heading >= 315 || heading < 45)
            {
                dir = "North";
            }
            else
            {
                if (heading >= 45 && heading < 135)
                {
                    dir = "East";
                }
                else
                {
                    if (heading >= 135 && heading < 225)
                    {
                        dir = "South";
                    }
                    else
                    {
                        dir = "West";
                    }
                }
            }

            printf("Roll: %f, Pitch: %f, Heading: %f\n", roll, pitch, heading);

            printf("Direction: %s\n", dir);
        }
    }
}

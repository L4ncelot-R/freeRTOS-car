/**
 * @file    magnetometer_direction.h
 * @author  Woon Jun Wei
 * @brief   This file contains the functions to calculate the direction of
 *          the car using the accelerometer and magnetometer data
 *
 * @details The direction of the car is calculated using the roll, pitch and yaw
 *          The roll and pitch are calculated using the accelerometer data
 *          The yaw is calculated using the magnetometer data
 *          The roll, pitch and yaw are combined to calculate the direction
 *          of the car
 *
 *          The direction of the car is calculated using the complementary filter
 *          The complementary filter is used to combine the accelerometer
 *          and magnetometer data to calculate the direction of the car
 *
 */

#ifndef MAGNETOMETER_DIRECTION_H
#define MAGNETOMETER_DIRECTION_H

#include "magnetometer_init.h"

/**
 * @brief Roll Calculation with Accelerometer Data
 * @param acceleration Accelerometer Data
 * @return
 */
float calculate_roll(int16_t acceleration[3]) {
    return atan2(acceleration[1], acceleration[2]) * (180.0 / M_PI);
}

/**
 * @brief Pitch Calculation with Accelerometer Data
 * @param acceleration Accelerometer Data
 * @return
 */
float calculate_pitch(int16_t acceleration[3]) {
    return atan2(- acceleration[0], sqrt(acceleration[1] * acceleration[1] +
                                         acceleration[2] * acceleration[2])) *
           (180.0 / M_PI);
}

/**
 * @brief Yaw Calculation with Magnetometer Data
 * @param magnetometer Magnetometer Data
 * @return
 */
float calculate_yaw_magnetometer(int16_t magnetometer[3]) {
    return atan2(magnetometer[1], magnetometer[0]) * (180.0 / M_PI);
}

/**
 * @brief Complementary Filter for Yaw
 * @param alpha     Complementary Filter Constant
 * @param yaw_acc   Yaw calculated from Accelerometer Data
 * @param yaw_mag   Yaw calculated from Magnetometer Data
 * @return yaw      Yaw calculated from Complementary Filter
 */
float calculate_yaw_complementary(float alpha, float yaw_acc, float yaw_mag) {
    return alpha * yaw_acc + (1 - alpha) * yaw_mag;
}

/**
 * @brief Adjust Yaw to be between 0 and 360 degrees
 * @param yaw   Yaw calculated from Complementary Filter
 * @return yaw  Yaw adjusted to be between 0 and 360 degrees
 */
float adjust_yaw(float yaw) {
    return (yaw < 0) ? yaw + 360.0f : yaw;
}

/**
 * @brief Calculate the Compass Direction (N, NE, E, SE, S, SW, W, NW)
 * 22.5 = 360 / 16, used to calculate the compass direction from
 * the compass direction enum
 * @param yaw   Yaw calculated from Complementary Filter
 * @return      Compass Direction
 */
compass_direction_t calculate_compass_direction(float yaw) {
    int orientation = (int) ((yaw + 22.5) / 45.0) % 8; // 8 compass directions
    switch (orientation)
    {
        case 0:
            return NORTH;
        case 1:
            return NORTH_EAST;
        case 2:
            return EAST;
        case 3:
            return SOUTH_EAST;
        case 4:
            return SOUTH;
        case 5:
            return SOUTH_WEST;
        case 6:
            return WEST;
        case 7:
            return NORTH_WEST;
        default:
            return NORTH;
    }
}

/**
 * @brief Update the Orientation Data
 * @param roll                  Roll calculated from Accelerometer Data
 * @param pitch                 Pitch calculated from Accelerometer Data
 * @param yaw                   Yaw calculated from Complementary Filter
 * @param compass_direction     Compass Direction
 */
void update_orientation_data(float roll, float pitch, float yaw,
                             compass_direction_t compass_direction) {
    g_direction.roll = roll;
    g_direction.pitch = pitch;
    g_direction.yaw = yaw;
    g_direction.orientation = compass_direction;
}

/**
 * @brief Read the Accelerometer and Magnetometer Data and
 *       Calculate the Direction of the Car
 * @details Alpha is set to 0.98 to give more weight to the accelerometer data
 * @param acceleration  Accelerometer Data
 * @param magnetometer  Magnetometer Data
 */
static void read_direction(int16_t acceleration[3], int16_t magnetometer[3]) {
    float roll = calculate_roll(acceleration);
    float pitch = calculate_pitch(acceleration);
    float yaw_mag = calculate_yaw_magnetometer(magnetometer);

    float alpha = 0.98;
    float yaw_acc = atan2(acceleration[1], acceleration[0]) * (180.0 / M_PI);
    float yaw = calculate_yaw_complementary(alpha, yaw_acc, yaw_mag);

    yaw = adjust_yaw(yaw);

    compass_direction_t compass_direction = calculate_compass_direction(yaw);

    update_orientation_data(roll, pitch, yaw, compass_direction);
}

/**
 * FreeRTOS Tasks
 */

/**
 * @brief Task to Monitor the Direction of the Car
 * @param params
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

            read_direction(accelerometer, magnetometer);

            printf("Roll: %f, Pitch: %f, Yaw: %f\n",
                   g_direction.roll, g_direction.pitch, g_direction.yaw);

            printf("Direction:");

            switch (g_direction.orientation)
            {
                case NORTH:
                    printf("North\n");
                    break;
                case NORTH_EAST:
                    printf("North East\n");
                    break;
                case EAST:
                    printf("East\n");
                    break;
                case SOUTH_EAST:
                    printf("South East\n");
                    break;
                case SOUTH:
                    printf("South\n");
                    break;
                case SOUTH_WEST:
                    printf("South West\n");
                    break;
                case WEST:
                    printf("West\n");
                    break;
                case NORTH_WEST:
                    printf("North West\n");
                    break;
            }

            switch (g_direction.roll_angle)
            {
                case LEFT:
                    printf("You're Flying!\n");
                    break;
                case RIGHT:
                    printf("You're Plunging!\n");
                    break;
            }

            switch (g_direction.pitch_angle)
            {
                case UP:
                    printf("Your left wheel is in the air!\n");
                    break;
                case DOWN:
                    printf("Your right wheel is in the air!\n");
                    break;
            }

        }
    }
}

#endif
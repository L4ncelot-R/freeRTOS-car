/**
 * @file    magnetometer_direction.h
 * @author  Woon Jun Wei
 * @brief   This file contains the functions to calculate the direction of
 *          the car using the accelerometer and magnetometer data
 *
 * @details The direction of the car is calculated using the roll, pitch and yaw
 *          The roll and pitch are calculated using the accelerometer data
 *          The yaw is calculated using the magnetometer and accelerometer data
 *          The roll, pitch and yaw are combined to calculate the direction
 *          of the car with a complementary filter and compensating for the
 *          temperature.
 *
 *          The complementary filter is used to combine the accelerometer
 *          and magnetometer data (yaw) to calculate the direction of the car
 *
 *          Source:
 *          https://www.nxp.com/docs/en/application-note/AN3461.pdf
 *          https://ahrs.readthedocs.io/en/latest/filters/complementary.html
 *
 */

#ifndef MAGNETOMETER_DIRECTION_H
#define MAGNETOMETER_DIRECTION_H

#include "magnetometer_init.h"
#include "map.h"

/**
 * @brief Roll Calculation with Accelerometer Data
 * @param acceleration Accelerometer Data
 * @return
 */
static inline float
calculate_roll(int16_t acceleration[3])
{
    return atan2(acceleration[1], acceleration[2]) * (180.0 / M_PI);
}

/**
 * @brief Pitch Calculation with Accelerometer Data
 * @param acceleration Accelerometer Data
 * @return
 */
static inline float
calculate_pitch(int16_t acceleration[3])
{
    return atan2(-acceleration[0],
                 sqrt((acceleration[1] * acceleration[1])
                      + (acceleration[2] * acceleration[2])))
           * (180.0 / M_PI);
}

/**
 * @brief Yaw Calculation with Magnetometer Data
 * @param magnetometer Magnetometer Data
 * @return
 */
static inline float
calculate_yaw_magnetometer(int16_t magnetometer[3])
{
    return atan2(magnetometer[1], magnetometer[0]) * (180.0f / M_PI);
}

/**
 * @brief Complementary Filter for Yaw
 * @param yaw_acc   Yaw calculated from Accelerometer Data
 * @param yaw_mag   Yaw calculated from Magnetometer Data
 * @return yaw      Yaw calculated from Complementary Filter
 */
// static inline float
// calculate_yaw_complementary(float yaw_acc, float yaw_mag) {
//     return ALPHA * yaw_acc + (1 - ALPHA) * yaw_mag;
// }

/**
 * @brief Compensate the magnetometer readings for temperature
 * @param yaw_mag       Yaw calculated from Magnetometer Data
 * @param temperature   Temperature in degrees Celsius
 * @return              Compensated Yaw
 */
float
compensate_magnetometer(float yaw_mag, int16_t temperature)
{
    // Calculate temperature difference from the reference temperature
    uint delta_temp = temperature - TEMPERATURE_OFFSET;

    // Apply temperature compensation to each axis using macros
    float compensated_yaw_mag
        = yaw_mag - ((float)delta_temp * TEMPERATURE_COEFFICIENT_Z);

    // Apply scale and offset corrections using macros
    compensated_yaw_mag = (compensated_yaw_mag - OFFSET_Z) * SCALE_Z;

    return compensated_yaw_mag;
}

/**
 * @brief Adjust Yaw to be between 0 and 360 degrees
 * @param yaw   Yaw calculated from Complementary Filter
 * @return yaw  Yaw adjusted to be between 0 and 360 degrees
 */
static inline float
adjust_yaw(float yaw)
{
    if (yaw < 0)
    {
        yaw += 360;
    }

    if (yaw > 360)
    {
        yaw -= 360;
    }

    return yaw;
}

/**
 * @brief Calculate the Compass Direction (N, NE, E, SE, S, SW, W, NW)
 * 22.5 = 360 / 16, used to calculate the compass direction from
 * the compass direction enum
 * 45.0 = 360 / 8, used to calculate the compass direction from
 * the orientation (0 - 7)
 * @param yaw   Yaw calculated
 * @return      Compass Direction
 */
static inline compass_direction_t
calculate_compass_direction(float yaw)
{
    if (yaw >= 337.5 || yaw < 22.5)
    {
        return NORTH;
    }
    else
    {
        if (yaw >= 22.5 && yaw < 67.5)
        {
            return NORTH_EAST;
        }
        else
        {
            if (yaw >= 67.5 && yaw < 112.5)
            {
                return EAST;
            }
            else
            {
                if (yaw >= 112.5 && yaw < 157.5)
                {
                    return SOUTH_EAST;
                }
                else
                {
                    if (yaw >= 157.5 && yaw < 202.5)
                    {
                        return SOUTH;
                    }
                    else
                    {
                        if (yaw >= 202.5 && yaw < 247.5)
                        {
                            return SOUTH_WEST;
                        }
                        else
                        {
                            if (yaw >= 247.5 && yaw < 292.5)
                            {
                                return WEST;
                            }
                            else
                            {
                                if (yaw >= 292.5 && yaw < 337.5)
                                {
                                    return NORTH_WEST;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief Update the Orientation Data
 * @param roll                  Roll calculated from Accelerometer Data
 * @param pitch                 Pitch calculated from Accelerometer Data
 * @param yaw                   Yaw calculated from Complementary Filter
 * @param compass_direction     Compass Direction
 */
static inline void
update_orientation_data(float                 roll,
                        float                 pitch,
                        float                 yaw,
                        compass_direction_t   compass_direction,
                        volatile direction_t *g_direction)
{
    g_direction->roll        = roll;
    g_direction->roll_angle  = (roll > 0) ? LEFT : RIGHT;
    g_direction->pitch       = pitch;
    g_direction->pitch_angle = (pitch > 0) ? UP : DOWN;
    g_direction->yaw         = yaw;
    g_direction->orientation = compass_direction;
}

/**
 * @brief Read the Accelerometer and Magnetometer Data and
 *       Calculate the Direction of the Car
 * @param acceleration  Accelerometer Data
 * @param magnetometer  Magnetometer Data
 */
static void
read_direction(int16_t               acceleration[3],
               int16_t               magnetometer[3],
               volatile direction_t *g_direction)
{

    float roll    = calculate_roll(acceleration);
    float pitch   = calculate_pitch(acceleration);
    float yaw_mag = calculate_yaw_magnetometer(magnetometer);

    yaw_mag = adjust_yaw(yaw_mag);

    compass_direction_t compass_direction
        = calculate_compass_direction(yaw_mag);

    update_orientation_data(
        roll, pitch, yaw_mag, compass_direction, g_direction);
}

/**
 * FreeRTOS Tasks
 */

/**
 * @brief Task to Monitor the Direction of the Car
 * @param params
 */
void
print_orientation_data(volatile direction_t g_direction)
{
    //    printf("Roll: %f, Pitch: %f, Yaw: %f\n",
    printf("%f %f %f\n", g_direction.roll, g_direction.pitch, g_direction.yaw);
}

void
print_direction(compass_direction_t direction)
{
    switch (direction)
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
}

void
print_roll_and_pitch(angle_t roll_angle, angle_t pitch_angle)
{
    switch (roll_angle)
    {
        case LEFT:
            printf("Your left wheel is in the air!\n");
            break;
        case RIGHT:
            printf("Your right wheel is in the air!\n");
            break;
    }

    switch (pitch_angle)
    {
        case UP:
            printf("You're Flying!\n");
            break;
        case DOWN:
            printf("You're Plunging!\n");
            break;
    }
}

void
updateDirection(volatile direction_t *g_direction)
{
    int16_t magnetometer[3];
    int16_t accelerometer[3];
    int16_t temperature[1];

    static int cur_x = 0;
    static int cur_y = 0;

    read_magnetometer(magnetometer, g_direction);
    read_accelerometer(accelerometer, g_direction);
    read_temperature(temperature);

    read_direction(accelerometer, magnetometer, g_direction);

    print_orientation_data(*g_direction);
}

void
monitor_direction_task(void *pvParameters)
{
    volatile direction_t *p_direction = NULL;
    p_direction                       = (direction_t *)pvParameters;

    for (;;)
    {
        updateDirection(p_direction);
        vTaskDelay(pdMS_TO_TICKS(DIRECTION_READ_DELAY));
    }
}

void
magnetometer_tasks_init(car_struct_t *car_struct)
{
    TaskHandle_t h_direction_task = NULL;
    xTaskCreate(monitor_direction_task,
                "Direction Task",
                configMINIMAL_STACK_SIZE,
                (void *)car_struct->p_direction,
                PRIO,
                &h_direction_task);
}

#endif
/*
 * @file motor_direction.c
 * @brief control the direction of the wheels by setting the GPIO mask
 * @author Richie
 */

#ifndef MOTOR_DIRECTION_H
#define MOTOR_DIRECTION_H

#include "motor_init.h"
#include "magnetometer_init.h"
#include "magnetometer_direction.h"
#include "math.h"

/*!
 * @brief Set the direction of the wheels; can use bitwise OR to set both
 * wheels such as DIRECTION_LEFT_FORWARD | DIRECTION_RIGHT_BACKWARD, it will
 * set the left motor to go forward and the right motor to go backward within
 * the same function.
 * if the motor direction is not set, it will not move.
 * @param direction The direction of the left and right wheels
 * @param left_speed The speed of the left motor, from 0.0 to 1.0
 * @param right_speed The speed of the right motor, from 0.0 to 1.0
 */
void
set_wheel_direction(uint32_t direction)
{
    gpio_put_masked(DIRECTION_MASK, 0U);
    gpio_set_mask(direction);
}

/*!
 * @brief Set the direction of the wheel to opposite direction using bit mask
 */
void
revert_wheel_direction()
{
    uint32_t current_direction = gpio_get_all();

    uint32_t reverted_direction = current_direction ^ DIRECTION_MASK;

    gpio_put_masked(DIRECTION_MASK, 0U);
    gpio_set_mask(reverted_direction & DIRECTION_MASK);
}

bool
check_direction(float current_direction, float target_direction, float range)
{
    // Normalize directions to be within 0 to 360 degrees
    current_direction = fmod(current_direction, 360.0f);
    if (current_direction < 0)
        current_direction += 360.0f;

    target_direction = fmod(target_direction, 360.0f);
    if (target_direction < 0)
        target_direction += 360.0f;

    // Check if current_direction is within ±1 degree of target_direction
    if (fabs(current_direction - target_direction) <= range
        || fabs(current_direction - target_direction) >= (360 - range))
    {
        return true;
    }
    return false;
}

void
spin_to_yaw(float target_yaw)
{
    updateDirection();
    float initial_yaw = g_direction.yaw;

    // if it will to turn more than 180 degrees, turn the other way
    if ((target_yaw > initial_yaw) && (target_yaw - initial_yaw < 180.f)
        || ((target_yaw < initial_yaw) && (initial_yaw - target_yaw >= 180.f)))
    {
        set_wheel_direction(DIRECTION_RIGHT);
    }
    else if ((target_yaw > initial_yaw) && (target_yaw - initial_yaw >= 180.f)
             || ((target_yaw < initial_yaw)
                 && (initial_yaw - target_yaw < 180.f)))
    {
        set_wheel_direction(DIRECTION_LEFT);
    }

    set_wheel_speed_synced(80u);

    g_use_pid = false;

    for (;;)
    {
        updateDirection();
        if (check_direction(g_direction.yaw, target_yaw, 1))
        {
            set_wheel_direction(DIRECTION_MASK);
            set_wheel_speed_synced(0u);
            break;
        }
    }

    g_use_pid = true;
    vTaskDelay(pdMS_TO_TICKS(50));
}

#endif /* MOTOR_DIRECTION_H */
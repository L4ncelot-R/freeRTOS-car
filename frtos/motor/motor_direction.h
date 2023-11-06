/*
 * @file motor_direction.c
 * @brief control the direction of the wheels by setting the GPIO mask
 * @author Richie
 */

#include "motor_init.h"

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

void
turn_left_90()
{
    set_wheel_direction(DIRECTION_RIGHT_FORWARD);
    set_wheel_speed(3500u, &g_motor_right);

    float initial = g_motor_right.speed.distance_cm;
    for (;;)
    {
        if (g_motor_right.speed.distance_cm - initial >= 17.f)
        {
            set_wheel_speed(0u, &g_motor_right);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    g_motor_right.speed.distance_cm = initial;
    g_motor_left.speed.distance_cm  = initial;
}

void
turn_right_90()
{
    set_wheel_direction(DIRECTION_LEFT_FORWARD);
    set_wheel_speed(3500u, &g_motor_left);

    float initial = g_motor_left.speed.distance_cm;
    for (;;)
    {
        if (g_motor_left.speed.distance_cm - initial >= 17.f)
        {
            set_wheel_speed(0u, &g_motor_left);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    g_motor_left.speed.distance_cm  = initial;
    g_motor_right.speed.distance_cm = initial;
}

void
spin_left_90()
{
    set_wheel_direction(DIRECTION_LEFT);

    set_wheel_speed_synced(3500u);

    float initial = g_motor_left.speed.distance_cm;
    for (;;)
    {
        if (g_motor_left.speed.distance_cm - initial >= 7.5f)
        {
            set_wheel_speed_synced(0u);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    g_motor_left.speed.distance_cm  = initial;
    g_motor_right.speed.distance_cm = initial;
}

void
spin_right_90()
{
    set_wheel_direction(DIRECTION_RIGHT);

    set_wheel_speed_synced(3500u);

    float initial = g_motor_right.speed.distance_cm;
    for (;;)
    {
        if (g_motor_right.speed.distance_cm - initial >= 7.5f)
        {
            set_wheel_speed_synced(0u);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    g_motor_right.speed.distance_cm = initial;
    g_motor_left.speed.distance_cm  = initial;
}
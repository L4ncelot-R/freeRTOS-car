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

/*!
 * @brief Check if the current direction is within the range of the target
 * @param current_direction current yaw
 * @param target_direction target yaw
 * @param range acceptable range
 * @return true if the current direction is within the range of the target
 */
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

/*!
 * @brief Spin the car to a certain yaw specifically
 * @param direction The direction to turn or spin
 * @param target_yaw The target yaw to spin to
 * @param pwm_level The pwm_level of the wheels, from 0 to 99
 * @param pp_car_struct The car struct pointer
 */
void
turn_to_yaw(uint32_t      direction,
            float         target_yaw,
            uint32_t      pwm_level,
            car_struct_t *pp_car_struct)
{
    pp_car_struct->p_pid->use_pid = false;

    set_wheel_direction(direction);

    set_wheel_speed_synced(pwm_level, pp_car_struct);

    for (;;)
    {
        updateDirection(pp_car_struct->p_direction);
        if (check_direction(pp_car_struct->p_direction->yaw, target_yaw, 1))
        {
            set_wheel_direction(DIRECTION_MASK);
            set_wheel_speed_synced(0u, pp_car_struct);
            break;
        }
    }

    pp_car_struct->p_pid->use_pid = true;
    vTaskDelay(pdMS_TO_TICKS(50));
}

/*!
 * @brief turn or spin the car to a certain degree offset from the current yaw
 * @param direction The direction to turn or spin
 * @param target_yaw The target yaw to spin to
 * @param pwm_level The pwm_level of the wheels, from 0 to 99
 * @param pp_car_struct The car struct pointer
 */
void
turn(uint32_t      direction,
     float         degree,
     uint32_t      pwm_level,
     car_struct_t *pp_car_struct)
{
    set_wheel_direction(DIRECTION_MASK);
    vTaskDelay(pdMS_TO_TICKS(50));

    updateDirection(pp_car_struct->p_direction);
    float initial_yaw = pp_car_struct->p_direction->yaw;
    float target_yaw  = (direction == DIRECTION_LEFT) ? initial_yaw - degree
                                                    : initial_yaw + degree;

    turn_to_yaw(direction, target_yaw, pwm_level, pp_car_struct);
}

#endif /* MOTOR_DIRECTION_H */
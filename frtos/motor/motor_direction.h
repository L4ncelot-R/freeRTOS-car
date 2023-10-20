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
set_wheel_direction (uint32_t direction)
{
    static const uint32_t mask = DIRECTION_LEFT_FORWARD |
                                 DIRECTION_LEFT_BACKWARD |
                                 DIRECTION_RIGHT_FORWARD |
                                 DIRECTION_RIGHT_BACKWARD;

    gpio_put_masked(mask, 0U);
    gpio_set_mask(direction);
}


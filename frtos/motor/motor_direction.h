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
    static const uint32_t mask
        = DIRECTION_LEFT_FORWARD | DIRECTION_LEFT_BACKWARD
          | DIRECTION_RIGHT_FORWARD | DIRECTION_RIGHT_BACKWARD;

    gpio_put_masked(mask, 0U);
    gpio_set_mask(direction);
}

/*!
 * @brief Turn the wheel, must set the priority higher than the motor PID task
 * @param direction The direction of the wheel
 * @param direction_after The direction of the wheel after turning
 * @param speed_after The speed of the wheel after turning
 */
void
turn_wheel(uint32_t direction)
{
    set_wheel_speed(0u);
    vTaskDelay(pdMS_TO_TICKS(1000));
    float initial_right = g_motor_right.speed.distance_cm;
    float initial_left  = g_motor_left.speed.distance_cm;

    set_wheel_direction(direction);
    set_wheel_speed(3500u);

    for (;;)
    {
        // gap between wheels = 11.3cm, to turn 90 degrees, need to travel
        // 11.3 * pi / 4 = 8.9cm
        if (g_motor_left.speed.distance_cm - initial_left >= 6.8f)
        {
            g_motor_left.pwm.level = 0;
            pwm_set_chan_level(g_motor_left.pwm.slice_num,
                               g_motor_left.pwm.channel,
                               g_motor_left.pwm.level);
        }

        if (g_motor_right.speed.distance_cm - initial_right >= 6.8f)
        {
            g_motor_right.pwm.level = 0;
            pwm_set_chan_level(g_motor_right.pwm.slice_num,
                               g_motor_right.pwm.channel,
                               g_motor_right.pwm.level);
        }

        if (g_motor_left.pwm.level == 0u && g_motor_right.pwm.level == 0u)
        {
            break;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
}
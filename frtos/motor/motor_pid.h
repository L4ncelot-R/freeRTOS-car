/*
 * @file motor_pid.h
 * @brief control the speed of the wheels by setting the PWM level, using PID
 * controller
 * @author Richie
 */

#include "motor_init.h"

/*!
 * @brief Compute the control signal using PID controller
 * @param target_speed The target speed of the wheel
 * @param current_speed The current speed of the wheel
 * @param integral The integral term of the PID controller
 * @param prev_error The previous error of the PID controller
 * @return The control signal
 */
float
compute_pid(float *integral, float *prev_error)
{
    float error
        = g_motor_left.speed.distance_cm - g_motor_right.speed.distance_cm;

    *integral += error;

    float derivative = error - *prev_error;

    float control_signal = g_motor_right.pid.kp_value * error
                           + g_motor_right.pid.ki_value * (*integral)
                           + g_motor_right.pid.kd_value * derivative;

    *prev_error = error;

    return control_signal;
}

void
motor_pid_task(__unused void *p_param)
{
    float integral   = 0.0f;
    float prev_error = 0.0f;

    for (;;)
    {
        if (g_motor_left.pwm.level == 0u)
        {
            g_motor_right.pwm.level = 0;
            pwm_set_chan_level(g_motor_right.pwm.slice_num,
                               g_motor_right.pwm.channel,
                               g_motor_right.pwm.level);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        float control_signal = compute_pid(&integral, &prev_error);

        float temp = (float) g_motor_right.pwm.level + control_signal * 0.05f;

        if (temp > MAX_SPEED)
        {
            temp = MAX_SPEED;
        }

        if (temp < MIN_SPEED)
        {
            temp = MIN_SPEED;
        }

        g_motor_right.pwm.level = (uint16_t) temp;

        pwm_set_chan_level(g_motor_right.pwm.slice_num,
                           g_motor_right.pwm.channel,
                           g_motor_right.pwm.level);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
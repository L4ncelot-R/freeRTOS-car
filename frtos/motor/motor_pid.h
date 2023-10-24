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
compute_pid(const volatile float *target_speed,
            const volatile float *current_speed,
            float                *integral,
            float                *prev_error)
{
    float error = *target_speed - *current_speed;
    *integral += error;

    float derivative = error - *prev_error;

    float control_signal
        = PID_KP * error + PID_KI * (*integral) + PID_KD * derivative;

    *prev_error = error;

    return control_signal;
}

void
motor_pid_task(void *p_param)
{
    motor_speed_t *p_motor_speed = p_param;
    float         integral       = 0.0f;
    float         prev_error     = 0.0f;

    for (;;)
    {
        float control_signal = compute_pid(&(p_motor_speed->target_speed_cms),
                                           &(p_motor_speed->current_speed_cms),
                                           &integral, &prev_error);

        if (p_motor_speed->pwm_level + control_signal > MAX_SPEED)
        {
            p_motor_speed->pwm_level = MAX_SPEED;
        }
        else if (p_motor_speed->pwm_level + control_signal < MIN_SPEED)
        {
            p_motor_speed->pwm_level = MIN_SPEED;
        }
        else
        {
            p_motor_speed->pwm_level = p_motor_speed->pwm_level + control_signal;
        }

        // printf("control signal: %f\n", control_signal);
        // printf("new pwm: %hu\n\n", p_motor_speed->pwm_level);

        pwm_set_chan_level(p_motor_speed->slice_num,
                           p_motor_speed->pwm_channel,
                           p_motor_speed->pwm_level);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
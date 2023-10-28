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
compute_pid(const volatile motor_t *p_motor,
            float                *integral,
            float                *prev_error)
{
    float error = p_motor->speed.target_cms - p_motor->speed.current_cms;

    *integral += error;

    float derivative = error - *prev_error;

    float control_signal
        = p_motor->pid.kp_value * error +
          p_motor->pid.ki_value * (*integral) +
          p_motor->pid.kd_value * derivative;

    *prev_error = error;

    return control_signal;
}

void
motor_pid_task(void *p_param)
{
    motor_t *p_motor   = p_param;
    float   integral   = 0.0f;
    float   prev_error = 0.0f;

    for (;;)
    {
        if (p_motor->speed.target_cms == 0.0f)
        {
            p_motor->pwm.level = 0;
            pwm_set_chan_level(p_motor->pwm.slice_num,
                               p_motor->pwm.channel,
                               p_motor->pwm.level);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        float control_signal = compute_pid(p_motor, &integral, &prev_error);

        if (p_motor->pwm.level + control_signal > MAX_SPEED)
        {
            p_motor->pwm.level = MAX_SPEED;
        }
        else if (p_motor->pwm.level + control_signal < MIN_SPEED)
        {
            p_motor->pwm.level = MIN_SPEED;
        }
        else
        {
            p_motor->pwm.level = p_motor->pwm.level + control_signal;
        }

        // printf("control signal: %f\n", control_signal);
        // printf("new pwm: %hu\n\n", p_motor_speed->level);

        pwm_set_chan_level(p_motor->pwm.slice_num,
                           p_motor->pwm.channel,
                           p_motor->pwm.level);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
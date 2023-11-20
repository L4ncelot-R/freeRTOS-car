/*
 * @file motor_pid.h
 * @brief control the speed of the wheels by setting the PWM level, using PID
 * controller
 * @author Richie
 */

#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include "magnetometer_init.h"
#include "magnetometer_direction.h"

/*!
 * @brief Compute the control signal using PID controller
 * @param integral The integral term of the PID controller
 * @param prev_error The previous error of the PID controller
 * @param car_struct The car_struct pointer
 * @return The control signal
 */
float
compute_pid(float *integral, float *prev_error, car_struct_t *car_struct)
{
    float error = car_struct->p_left_motor->speed.distance_cm
                  - car_struct->p_right_motor->speed.distance_cm;

    *integral += error;

    float derivative = error - *prev_error;

    float control_signal
        = car_struct->p_pid->kp_value * error
          + car_struct->p_pid->ki_value * (*integral)
          + car_struct->p_pid->kd_value * derivative;

    *prev_error = error;

    return control_signal;
}


/*!
 * @brief Repeating timer handler for the PID controller
 * @param ppp_timer The repeating timer
 * @return true
 */
bool
repeating_pid_handler(struct repeating_timer *ppp_timer)
{
    car_struct_t *car_strut = (car_struct_t *)ppp_timer->user_data;

    static float integral   = 0.0f;
    static float prev_error = 0.0f;

    if (!car_strut->p_pid->use_pid)
    {
        return true;
    }

    float control_signal = compute_pid(&integral, &prev_error, car_strut);

    float temp
        = (float)car_strut->p_right_motor->pwm.level + control_signal * 0.05f;

    if (temp > MAX_PWM_LEVEL)
    {
        temp = MAX_PWM_LEVEL;
    }

    if (temp <= MIN_PWM_LEVEL)
    {
        temp = MIN_PWM_LEVEL + 1u;
    }

    set_wheel_speed((uint32_t)temp, car_strut->p_right_motor);

    return true;
}

#endif /* MOTOR_PID_H */
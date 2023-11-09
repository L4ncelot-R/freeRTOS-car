/*
 * @file motor_pid.h
 * @brief control the speed of the wheels by setting the PWM level, using PID
 * controller
 * @author Richie
 */

#ifndef MOTOR_PID_H
#define MOTOR_PID_H

// #include "magnetometer_init.h"

/*!
 * @brief Compute the control signal using PID controller
 * @param target_speed The target speed of the wheel
 * @param current_speed The current speed of the wheel
 * @param integral The integral term of the PID controller
 * @param prev_error The previous error of the PID controller
 * @return The control signal
 */
float
compute_pid(float *integral, float *prev_error, car_struct_t *car_struct)
{
    float error = car_struct->p_left_motor->speed.distance_cm
                  - car_struct->p_right_motor->speed.distance_cm;

    printf("%f\n", error);

    *integral += error;

    float derivative = error - *prev_error;

    float control_signal
        = car_struct->p_right_motor->p_pid->kp_value * error
          + car_struct->p_right_motor->p_pid->ki_value * (*integral)
          + car_struct->p_right_motor->p_pid->kd_value * derivative;

    *prev_error = error;

    return control_signal;
}

bool
repeating_pid_handler(struct repeating_timer *t)
{
    car_struct_t *car_strut = (car_struct_t *)t->user_data;

    static float integral   = 0.0f;
    static float prev_error = 0.0f;

    if (!car_strut->p_right_motor->p_pid->use_pid)
    {
        return true;
    }

    float control_signal = compute_pid(&integral, &prev_error, car_strut);

    printf("control: %f\n", control_signal);

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

    printf("temp: %f\n", temp);

    set_wheel_speed((uint32_t)temp, car_strut->p_right_motor);

    printf("speed: %f cm/s\n", car_strut->p_right_motor->speed.current_cms);
    printf("distance: %f cm\n", car_strut->p_right_motor->speed.distance_cm);

    return true;
}

#endif /* MOTOR_PID_H */
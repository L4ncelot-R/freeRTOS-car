/*
 * @file motor_pid.h
 * @brief control the speed of the wheels by setting the PWM level, using PID
 * controller
 * @author Richie
 */

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
compute_pid(float *integral, float *prev_error)
{
    float error
        = g_motor_left.speed.distance_cm - g_motor_right.speed.distance_cm;

    printf("%f\n", error);

    *integral += error;

    float derivative = error - *prev_error;

    float control_signal = g_motor_right.pid.kp_value * error
                           + g_motor_right.pid.ki_value * (*integral)
                           + g_motor_right.pid.kd_value * derivative;

    *prev_error = error;

    return control_signal;
}

bool
repeating_pid_handler(__unused struct repeating_timer *t)
{
    static float integral   = 0.0f;
    static float prev_error = 0.0f;

    if (!g_use_pid)
    {
        return true;
    }

    float control_signal = compute_pid(&integral, &prev_error);

    float temp = (float)g_motor_right.pwm.level + control_signal * 0.05f;

    if (temp > MAX_PWM_LEVEL)
    {
        temp = MAX_PWM_LEVEL;
    }

    if (temp <= MIN_PWM_LEVEL)
    {
        temp = MIN_PWM_LEVEL + 1u;
    }

    g_motor_right.pwm.level = (uint16_t)temp;
    pwm_set_chan_level(g_motor_right.pwm.slice_num,
                       g_motor_right.pwm.channel,
                       g_motor_right.pwm.level);

//    printf("speed: %f cm/s\n", g_motor_right.speed.current_cms);
//    printf("distance: %f cm\n", g_motor_right.speed.distance_cm);

    return true;
}
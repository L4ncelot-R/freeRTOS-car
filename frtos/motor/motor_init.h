/*
 * @file motor_init.h
 * @brief define the constants and initialize the motor
 * @author Richie
 */

#ifndef MOTOR_INIT_H
#define MOTOR_INIT_H

#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "motor_config.h"

/*!
 * @brief Initialize the motor
 * @param car_struct The car_struct. Need to have the following fields:\n
 *                  - p_left_motor\n
 *                  - p_right_motor\n
 *                  - p_pid
 */
void
motor_init(car_struct_t *car_struct)
{
    // Semaphore
    g_left_sem  = xSemaphoreCreateBinary();
    g_right_sem = xSemaphoreCreateBinary();

    car_struct->p_pid->use_pid  = true;
    car_struct->p_pid->kp_value = 600.f;
    car_struct->p_pid->ki_value = 66.67f;
    car_struct->p_pid->kd_value = 1350.f;

    // initialize the car_struct
    car_struct->p_left_motor->pwm.level         = 0u;
    car_struct->p_left_motor->pwm.channel       = PWM_CHAN_A;
    car_struct->p_left_motor->speed.distance_cm = 0.0f;
    car_struct->p_left_motor->p_sem             = &g_left_sem;
    car_struct->p_left_motor->use_pid           = &car_struct->p_pid->use_pid;

    car_struct->p_right_motor->pwm.level         = 0u;
    car_struct->p_right_motor->pwm.channel       = PWM_CHAN_B;
    car_struct->p_right_motor->speed.distance_cm = 0.0f;
    car_struct->p_right_motor->p_sem             = &g_right_sem;
    car_struct->p_right_motor->use_pid           = &car_struct->p_pid->use_pid;

    // Initialize speed pins as inputs
    gpio_init(SPEED_PIN_RIGHT);
    gpio_init(SPEED_PIN_LEFT);
    gpio_set_dir(SPEED_PIN_RIGHT, GPIO_IN);
    gpio_set_dir(SPEED_PIN_LEFT, GPIO_IN);

    // Initialize direction pins as outputs
    gpio_init(DIRECTION_PIN_RIGHT_IN1);
    gpio_init(DIRECTION_PIN_RIGHT_IN2);
    gpio_init(DIRECTION_PIN_LEFT_IN3);
    gpio_init(DIRECTION_PIN_LEFT_IN4);

    gpio_set_dir(DIRECTION_PIN_RIGHT_IN1, GPIO_OUT);
    gpio_set_dir(DIRECTION_PIN_RIGHT_IN2, GPIO_OUT);
    gpio_set_dir(DIRECTION_PIN_LEFT_IN3, GPIO_OUT);
    gpio_set_dir(DIRECTION_PIN_LEFT_IN4, GPIO_OUT);

    // Initialise PWM
    gpio_set_function(PWM_PIN_LEFT, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN_RIGHT, GPIO_FUNC_PWM);

    car_struct->p_left_motor->pwm.slice_num
        = pwm_gpio_to_slice_num(PWM_PIN_LEFT);
    car_struct->p_right_motor->pwm.slice_num
        = pwm_gpio_to_slice_num(PWM_PIN_RIGHT);

    // NOTE: PWM clock is 125MHz for raspberrypi pico w by default

    // 125MHz / 50 = 2500kHz
    pwm_set_clkdiv(car_struct->p_left_motor->pwm.slice_num, PWM_CLK_DIV);
    pwm_set_clkdiv(car_struct->p_right_motor->pwm.slice_num, PWM_CLK_DIV);

    // L289N can accept up to 40kHz
    // 2500kHz / 100 = 25kHz
    pwm_set_wrap(car_struct->p_left_motor->pwm.slice_num, (PWM_WRAP - 1U));
    pwm_set_wrap(car_struct->p_right_motor->pwm.slice_num, (PWM_WRAP - 1U));

    pwm_set_enabled(car_struct->p_left_motor->pwm.slice_num, true);
    pwm_set_enabled(car_struct->p_right_motor->pwm.slice_num, true);
}

#endif /* MOTOR_INIT_H */
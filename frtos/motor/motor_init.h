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

uint g_slice_num_left  = 0U;
uint g_slice_num_right = 0U;

SemaphoreHandle_t g_wheel_speed_sem_left  = NULL;
SemaphoreHandle_t g_wheel_speed_sem_right = NULL;

motor_speed_t g_motor_speed_left = { .target_speed_cms = 0.0f,
                                     .pwm_level        = 2500u,
                                     .p_sem       = &g_wheel_speed_sem_left,
                                     .p_slice_num = &g_slice_num_left,
                                     .pwm_channel = PWM_CHAN_A };

motor_speed_t g_motor_speed_right = { .target_speed_cms = 0.0f,
                                      .pwm_level        = 2500u,
                                      .p_sem       = &g_wheel_speed_sem_right,
                                      .p_slice_num = &g_slice_num_right,
                                      .pwm_channel = PWM_CHAN_B };

void
motor_init(void)
{
    // Semaphore
    g_wheel_speed_sem_left  = xSemaphoreCreateBinary();
    g_wheel_speed_sem_right = xSemaphoreCreateBinary();

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

    g_slice_num_left  = pwm_gpio_to_slice_num(PWM_PIN_LEFT);
    g_slice_num_right = pwm_gpio_to_slice_num(PWM_PIN_RIGHT);

    // NOTE: PWM clock is 125MHz for raspberrypi pico w by default

    // 125MHz / 250 = 500kHz
    pwm_set_clkdiv(g_slice_num_left, PWM_CLK_DIV);
    pwm_set_clkdiv(g_slice_num_right, PWM_CLK_DIV);

    // have them to be 500kHz / 5000 = 100Hz
    pwm_set_wrap(g_slice_num_left, (PWM_WRAP - 1U));
    pwm_set_wrap(g_slice_num_right, (PWM_WRAP - 1U));

    pwm_set_enabled(g_slice_num_left, true);
    pwm_set_enabled(g_slice_num_right, true);
}

#endif /* MOTOR_INIT_H */
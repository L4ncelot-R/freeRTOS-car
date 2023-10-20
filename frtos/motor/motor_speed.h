/*
 * @file motor_speed.c
 * @brief control the speed of the wheels by setting the PWM level, and
 * monitor the speed by using edge interrupt and measure the time between
 * each slot of the wheel, then calculate the speed of the wheel in cm/s, and
 * adjust the speed of the wheel by using PID controller, and set the new PWM
 * @author Richie
 */

#include "motor_init.h"

/*!
 * @brief Set the speed of the wheels; can use bitwise OR to set both
 * @param speed in range [0, 5000]
 * @param side 0 for left, 1 for right
 */
void
set_wheel_speed(float speed, uint8_t side)
{
    if (side == 0U)
    {
        pwm_set_chan_level(g_slice_num_left, PWM_CHAN_A, (uint16_t)speed);
    }
    else
    {
        pwm_set_chan_level(g_slice_num_right, PWM_CHAN_B, (uint16_t)speed);
    }
}

void
h_left_wheel_sensor_isr_handler(void)
{
    if (gpio_get_irq_event_mask(SPEED_PIN_LEFT) & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(SPEED_PIN_LEFT, GPIO_IRQ_EDGE_FALL);

        // printf("left motor sensor isr\n");
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_wheel_speed_sem_left,
                              &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void
h_right_wheel_sensor_isr_handler(void)
{
    if (gpio_get_irq_event_mask(SPEED_PIN_RIGHT) & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL);

        // printf("right motor sensor isr\n");
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_wheel_speed_sem_right,
                              &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

float
compute_pid(float  target_speed,
            float  current_speed,
            float *integral,
            float *prev_error)
{
    float error = target_speed - current_speed;
    *integral += error;

    float derivative = error - *prev_error;

    float control_signal
        = PID_KP * error + PID_KI * (*integral) + PID_KD * derivative;

    *prev_error = error;

    return control_signal;
}

void
monitor_left_wheel_speed_task(void *pvParameters)
{
    //  static float * target_speed = NULL;
    //  *target_speed = * (float *) pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(g_wheel_speed_sem_left, portMAX_DELAY) == pdTRUE)
        {
            static uint64_t curr_time_left = 0u;
            curr_time_left                 = time_us_64();

            static uint64_t prev_time_left    = 0u;
            static uint64_t elapsed_time_left = 1u; // to avoid division by 0

            elapsed_time_left = curr_time_left - prev_time_left;

            prev_time_left = curr_time_left;

            static float speed_left = 0.f;
            // speed in cm/s; speed = distance / time
            // distance = circumference / 20
            // circumference = 2 * pi * 3.25 cm = 20.4203522483 cm
            // distance = 20.4203522483 cm / 20 = 1.02101761242 cm
            speed_left
                = (float)(1.02101761242f / (elapsed_time_left / 1000000.f));

            printf("left speed: %f cm/s\n", speed_left);

            static float control_signal = 0.f;
            static float integral       = 0.f;
            static float prev_error     = 0.f;

            control_signal = compute_pid(
                *(float *)pvParameters, speed_left, &integral, &prev_error);

            static float new_pwm = START_SPEED;

            if (new_pwm + control_signal > MAX_SPEED)
            {
                new_pwm = MAX_SPEED;
            }
            else if (new_pwm + control_signal < MIN_SPEED)
            {
                new_pwm = MIN_SPEED;
            }
            else
            {
                new_pwm = new_pwm + control_signal;
            }

            printf("control signal: %f\n", control_signal);
            printf("new pwm: %f\n\n", new_pwm);

            set_wheel_speed(new_pwm, 0u);
        }
    }
}

void
monitor_right_wheel_speed_task(void *pvParameters)
{
    // volatile float * target_speed = (float *) pvParameters;

    //  static volatile float * target_speed = NULL;
    //  target_speed = (float *) pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(g_wheel_speed_sem_right, portMAX_DELAY) == pdTRUE)
        {
            static uint64_t curr_time_right = 0u;
            curr_time_right                 = time_us_64();

            static uint64_t prev_time_right    = 0u;
            static uint64_t elapsed_time_right = 1u; // to avoid division by 0

            elapsed_time_right = curr_time_right - prev_time_right;

            prev_time_right = curr_time_right;

            static float speed_right = 0.f;

            speed_right
                = (float)(1.02101761242f / (elapsed_time_right / 1000000.f));

            printf("right speed: %f cm/s\n", speed_right);

            static float control_signal = 0.f;
            static float integral       = 0.f;
            static float prev_error     = 0.f;

            control_signal = compute_pid(
                *(float *)pvParameters, speed_right, &integral, &prev_error);

            static float new_pwm = START_SPEED;

            if (new_pwm + control_signal > MAX_SPEED)
            {
                new_pwm = MAX_SPEED;
            }
            else if (new_pwm + control_signal < MIN_SPEED)
            {
                new_pwm = MIN_SPEED;
            }
            else
            {
                new_pwm = new_pwm + control_signal;
            }

            printf("control signal: %f\n", control_signal);
            printf("new pwm: %f\n\n", new_pwm);

            set_wheel_speed(new_pwm, 1u);
        }
    }
}
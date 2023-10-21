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
 * @brief Interrupt handler for the wheel sensor
 */
void
h_wheel_sensor_isr_handler(void)
{
    if (gpio_get_irq_event_mask(SPEED_PIN_LEFT) & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(SPEED_PIN_LEFT, GPIO_IRQ_EDGE_FALL);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_wheel_speed_sem_left,
                              &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (gpio_get_irq_event_mask(SPEED_PIN_RIGHT) & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_wheel_speed_sem_right,
                              &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

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
            const float          *current_speed,
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

/*!
 * @brief Task to monitor and control the speed of the wheel
 * @param pvParameters motor_speed_t struct pointer
 * @see motor_speed_t
 */
void
monitor_wheel_speed_task(void *pvParameters)
{
    volatile motor_speed_t *p_motor_speed = NULL;
    p_motor_speed                         = (motor_speed_t *)pvParameters;

    float speed = 0.f;

    float new_pwm = p_motor_speed->pwm_level;

    uint64_t curr_time    = 0u;
    uint64_t prev_time    = 0u;
    uint64_t elapsed_time = 0u;

    float control_signal = 0.f;
    float integral       = 0.f;
    float prev_error     = 0.f;

    for (;;)
    {
        if (xSemaphoreTake(*p_motor_speed->p_sem, pdMS_TO_TICKS(100))
            == pdTRUE)
        {
            curr_time    = time_us_64();
            elapsed_time = curr_time - prev_time;
            prev_time    = curr_time;

            // speed in cm/s; speed = distance / time
            // distance = circumference / 20
            // circumference = 2 * pi * 3.25 cm = 20.4203522483 cm
            // distance = 20.4203522483 cm / 20 = 1.02101761242 cm
            speed = (float)(1.02101761242f / (elapsed_time / 1000000.f));

            printf("speed: %f cm/s\n", speed);
        }
        else
        {
            speed = 0.f;
            printf("stopped\n");
        }

        control_signal = compute_pid(
            &(p_motor_speed->target_speed_cms), &speed, &integral, &prev_error);

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

        pwm_set_chan_level(*p_motor_speed->p_slice_num,
                           p_motor_speed->pwm_channel,
                           (int16_t)new_pwm);
    }
}
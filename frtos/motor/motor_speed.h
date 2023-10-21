/*
 * @file motor_speed.c
 * @brief monitor and update the speed of the wheels
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
 * @brief Task to monitor and control the speed of the wheel
 * @param pvParameters motor_speed_t struct pointer
 * @see motor_speed_t
 */
void
monitor_wheel_speed_task(void *pvParameters)
{
    volatile motor_speed_t *p_motor_speed = NULL;
    p_motor_speed                         = (motor_speed_t *)pvParameters;

    uint64_t curr_time    = 0u;
    uint64_t prev_time    = 0u;
    uint64_t elapsed_time = 0u;

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
            p_motor_speed->current_speed_cms = (float) (1.02101761242f /
                                                       (elapsed_time /
                                                        1000000.f));

            printf("speed: %f cm/s\n", p_motor_speed->current_speed_cms);
        }
        else
        {
            p_motor_speed->current_speed_cms = 0.f;
            printf("stopped\n");
        }
    }
}
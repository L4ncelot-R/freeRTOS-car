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
        xSemaphoreGiveFromISR(g_motor_left.sem,
                              &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (gpio_get_irq_event_mask(SPEED_PIN_RIGHT) & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_motor_right.sem,
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
    volatile motor_t *p_motor = NULL;
    p_motor                   = (motor_t *)pvParameters;

    uint64_t curr_time    = 0u;
    uint64_t prev_time    = 0u;
    uint64_t elapsed_time = 0u;

    for (;;)
    {
        if ((xSemaphoreTake(p_motor->sem, pdMS_TO_TICKS(100))
            == pdTRUE) && (g_use_pid == true))
        {
            curr_time    = time_us_64();
            elapsed_time = curr_time - prev_time;
            prev_time    = curr_time;

            // speed in cm/s; speed = distance / time
            // distance = circumference / 20
            // circumference = 2 * pi * 3.25 cm = 20.4203522483 cm
            // distance = 20.4203522483 cm / 20 = 1.02101761242 cm
            p_motor->speed.current_cms
                = (float) (1021017.61242f / elapsed_time);

            p_motor->speed.distance_cm += 1.02101761242f;
        }
        else
        {
            p_motor->speed.current_cms = 0.f;
        }
    }
}

void
set_wheel_speed(uint32_t pwm_level, motor_t * motor)
{
    motor->pwm.level = pwm_level;

    pwm_set_chan_level(motor->pwm.slice_num,
                       motor->pwm.channel,
                       motor->pwm.level);
}

///*!
// * @brief Set the speed of the wheels
// * @param pwm_level The pwm_level of the wheels, from 0 to 99
// */
//void
//set_wheel_speed_synced(uint32_t pwm_level)
//{
//    if (pwm_level > MAX_PWM_LEVEL)
//    {
//        pwm_level = MAX_PWM_LEVEL;
//    }
//
//    set_wheel_speed(pwm_level, &g_motor_left);
//    set_wheel_speed(pwm_level, &g_motor_right);
//}

///*!
// * @brief Set the distance to travel before stopping, must be called after
// * setting the speed and direction.
// * @param distance_cm distance to travel in cm
// */
//void
//distance_to_stop(float distance_cm)
//{
//    float initial = g_motor_right.speed.distance_cm;
//
//    for (;;)
//    {
//        if (g_motor_right.speed.distance_cm - initial >= distance_cm)
//        {
//            set_wheel_speed_synced(0u);
//            break;
//        }
//        vTaskDelay(pdMS_TO_TICKS(10));
//    }
//    vTaskDelay(pdMS_TO_TICKS(1000));
//    g_motor_right.speed.distance_cm = g_motor_left.speed.distance_cm;
//}

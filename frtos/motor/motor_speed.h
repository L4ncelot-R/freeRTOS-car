/*
 * @file motor_speed.c
 * @brief monitor and update the speed of the wheels
 * @author Richie
 */
#ifndef MOTOR_SPEED_H
#define MOTOR_SPEED_H

#include "motor_init.h"
#include "magnetometer_init.h"
#include "magnetometer_direction.h"

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
        xSemaphoreGiveFromISR(g_left_sem,
                              &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (gpio_get_irq_event_mask(SPEED_PIN_RIGHT) & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_right_sem,
                              &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


/*!
 * @brief Task to monitor and control the speed of the wheel
 * @param ppp_motor motor_speed_t struct pointer
 * @see motor_speed_t
 */
void
monitor_wheel_speed_task(void *ppp_motor)
{
    volatile motor_t *p_motor = NULL;
    p_motor                   = (motor_t *)ppp_motor;

    uint64_t curr_time    = 0u;
    uint64_t prev_time    = 0u;
    uint64_t elapsed_time = 0u;

    for (;;)
    {
        if ((xSemaphoreTake(*p_motor->p_sem, pdMS_TO_TICKS(100))
            == pdTRUE) && (*p_motor->use_pid == true))
        {
            curr_time    = time_us_64();
            elapsed_time = curr_time - prev_time;
            prev_time    = curr_time;

            p_motor->speed.current_cms
                = (float) (SLOT_DISTANCE_CM_MODIFIED / elapsed_time);

            p_motor->speed.distance_cm += SLOT_DISTANCE_CM;
        }
        else
        {
            p_motor->speed.current_cms = 0.f;
        }
    }
}

/*!
 * @brief Set the speed of the wheels
 * @param pwm_level The pwm_level of the wheels, from 0 to 99
 * @param p_motor The motor to set the speed
 */
void
set_wheel_speed(uint32_t pwm_level, motor_t *p_motor)
{
    if (pwm_level > MAX_PWM_LEVEL)
    {
        pwm_level = MAX_PWM_LEVEL;
    }

    p_motor->pwm.level = pwm_level;

    pwm_set_chan_level(
        p_motor->pwm.slice_num, p_motor->pwm.channel, p_motor->pwm.level);
}

/*!
 * @brief Set the speed of the wheels
 * @param pwm_level The pwm_level of the wheels, from 0 to 99
 * @param pp_car_strut The car struct pointer
 */
void
set_wheel_speed_synced(uint32_t pwm_level, car_struct_t *pp_car_strut)
{
    updateDirection(pp_car_strut->p_direction);
    pp_car_strut->p_direction->target_yaw = pp_car_strut->p_direction->yaw;

    set_wheel_speed(pwm_level, pp_car_strut->p_left_motor);
    set_wheel_speed(pwm_level, pp_car_strut->p_right_motor);
}

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

#endif /* MOTOR_SPEED_H */

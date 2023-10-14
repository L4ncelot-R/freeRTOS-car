/**
 * @brief Program to read onboard temperature sensor and print out the average
 */

#include <stdio.h>

// pico sdk libraries
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

// FreeRTOS libraries
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "message_buffer.h"
#include "semphr.h"

#include "wheel.h"

#define READ_LEFT_WHEEL_SPEED_PRIO (tskIDLE_PRIORITY + 1UL)
#define READ_RIGHT_WHEEL_SPEED_PRIO (tskIDLE_PRIORITY + 1UL)

void
launch()
{
    // isr to detect right wheel slot
    gpio_set_irq_enabled(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(SPEED_PIN_RIGHT,
                             h_right_wheel_sensor_isr_handler);

    // isr to detect left wheel slot
    gpio_set_irq_enabled(SPEED_PIN_LEFT, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(SPEED_PIN_LEFT,
                             h_left_wheel_sensor_isr_handler);

    irq_set_enabled(IO_IRQ_BANK0, true);

    static volatile float * p_target_speed = NULL;
    static volatile float target_speed  = 20.0f; // cm/s
    p_target_speed = &target_speed;

    TaskHandle_t h_monitor_left_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_left_wheel_speed_task,
                "monitor_left_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                (void *) p_target_speed,
                READ_LEFT_WHEEL_SPEED_PRIO,
                &h_monitor_left_wheel_speed_task_handle);

    TaskHandle_t h_monitor_right_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_right_wheel_speed_task,
                "monitor_right_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                (void *) p_target_speed,
                READ_RIGHT_WHEEL_SPEED_PRIO,
                &h_monitor_right_wheel_speed_task_handle);

    vTaskStartScheduler();
}

int
main (void)
{
    stdio_usb_init();
    wheel_setup();
    sleep_ms(5000);

    set_wheel_direction(DIRECTION_RIGHT_FORWARD);
    set_wheel_speed(START_SPEED, 1u);

    launch();

    while (1)
    {
    }

    return (0);
}

/*** end of file ***/

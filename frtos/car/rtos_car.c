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
    // isr to detect left wheel slot
    gpio_set_irq_enabled_with_callback(SPEED_PIN_RIGHT,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &right_wheel_sensor_isr);

    // isr to detect right wheel slot
    gpio_set_irq_enabled_with_callback(SPEED_PIN_LEFT,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &left_wheel_sensor_isr);

    TaskHandle_t h_monitor_left_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_left_wheel_speed_task,
                "monitor_left_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                NULL,
                READ_LEFT_WHEEL_SPEED_PRIO,
                &h_monitor_left_wheel_speed_task_handle);

    TaskHandle_t h_monitor_right_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_right_wheel_speed_task,
                "monitor_right_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                NULL,
                READ_RIGHT_WHEEL_SPEED_PRIO,
                &h_monitor_right_wheel_speed_task_handle);

    vTaskStartScheduler();
}

int
main (void)
{
    stdio_usb_init();
    wheel_setup();
    sleep_ms(2000);

    set_wheel_direction(DIRECTION_RIGHT_FORWARD);
    set_wheel_speed(1.f, 1u);

    launch();

    while (1)
    {
    }

    return (0);
}

/*** end of file ***/

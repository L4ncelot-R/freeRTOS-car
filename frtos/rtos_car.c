/**
 * @brief Program to read onboard temperature sensor and print out the average
 */

#include <stdio.h>

// pico sdk libraries
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

// FreeRTOS libraries
#include "FreeRTOS.h"
#include "task.h"

#include "motor_speed.h"
#include "motor_direction.h"

#include "line_sensor.h"

#define READ_LEFT_WHEEL_SPEED_PRIO      (tskIDLE_PRIORITY + 1UL)
#define READ_RIGHT_WHEEL_SPEED_PRIO     (tskIDLE_PRIORITY + 1UL)

#define READ_LEFT_SENSOR_PRIO           (tskIDLE_PRIORITY + 2UL)
#define READ_RIGHT_SENSOR_PRIO          (tskIDLE_PRIORITY + 2UL)

#define DIRECTION_TASK_PRIORITY         (tskIDLE_PRIORITY + 3UL)

/* Common Car State Structure (TODO: TBC)*/
//static car_state_t g_car_state;

void
launch()
{
    // isr to detect right motor slot
    gpio_set_irq_enabled(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(SPEED_PIN_RIGHT,
                             h_right_wheel_sensor_isr_handler);

    // isr to detect left motor slot
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

    struct repeating_timer g_left_sensor_timer;
    add_repeating_timer_ms(LINE_SENSOR_READ_DELAY,
                           h_left_sensor_timer_handler,
                           NULL,
                           &g_left_sensor_timer);

    struct repeating_timer g_right_sensor_timer;
    add_repeating_timer_ms(LINE_SENSOR_READ_DELAY,
                           h_right_sensor_timer_handler,
                           NULL,
                           &g_right_sensor_timer);

    TaskHandle_t h_monitor_left_sensor_task;
    xTaskCreate(monitor_left_sensor_task,
                "Monitor Left Sensor Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                READ_LEFT_SENSOR_PRIO,
                &h_monitor_left_sensor_task);

    TaskHandle_t h_monitor_right_sensor_task;
    xTaskCreate(monitor_right_sensor_task,
                "Monitor Right Sensor Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                READ_RIGHT_SENSOR_PRIO,
                &h_monitor_right_sensor_task);

    TaskHandle_t h_monitor_direction_task;
    xTaskCreate(monitor_direction_task,
                "Monitor Direction Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                DIRECTION_TASK_PRIORITY,
                &h_monitor_direction_task);

    vTaskStartScheduler();
}

int
main (void)
{
    stdio_usb_init();

    motor_init();

    set_wheel_direction(DIRECTION_LEFT_FORWARD | DIRECTION_RIGHT_FORWARD);

    line_sensor_setup();

    initialize_car_state(); // TODO: Could be common functionality, To confirm
                            // during Integration
    launch();

    return (0);
}

/*** end of file ***/

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
#include "motor_pid.h"

#include "line_sensor.h"

#include "ultrasonic_sensor.h"

#define READ_LEFT_SENSOR_PRIO    (tskIDLE_PRIORITY + 2UL)
#define READ_RIGHT_SENSOR_PRIO   (tskIDLE_PRIORITY + 2UL)
#define READ_BARCODE_SENSOR_PRIO (tskIDLE_PRIORITY + 2UL)

#define DIRECTION_TASK_PRIORITY (tskIDLE_PRIORITY + 3UL)

#define DISTANCE_TASK_PRIORITY (tskIDLE_PRIORITY + 4UL)

/* Common Car State Structure (TODO: TBC)*/
// static car_state_t g_car_state;

static void
motor_control_task(__unused void *p_param)
{
    static bool left = false;
    static bool right = false;

    for (;;)
    {
        state_t state = gpio_get(LEFT_SENSOR_PIN);
        printf("state: %d\n", state);

        if (state == 1)
        {
            if (!left)
            {
                spin_left_90();
                left = true;
            }
            else if (!right)
            {
                spin_right_90();
                spin_right_90();
                right = true;
            }
            else
            {
                spin_right_90();
                left = false;
                right = false;
            }
        }
        else
        {
            set_wheel_direction(DIRECTION_FORWARD);
            set_wheel_speed_synced(3500u);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void
launch()
{
//    // isr to detect left line sensor
//    gpio_set_irq_enabled(LEFT_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true);
//    gpio_add_raw_irq_handler(LEFT_SENSOR_PIN, h_line_sensor_handler);

    // isr to detect right motor slot
    gpio_set_irq_enabled(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(SPEED_PIN_RIGHT, h_wheel_sensor_isr_handler);

    // isr to detect left motor slot
    gpio_set_irq_enabled(SPEED_PIN_LEFT, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(SPEED_PIN_LEFT, h_wheel_sensor_isr_handler);

    irq_set_enabled(IO_IRQ_BANK0, true);

//    // line sensor timer
//    struct repeating_timer g_left_sensor_timer;
//    add_repeating_timer_ms(LINE_SENSOR_READ_DELAY,
//                           h_left_sensor_timer_handler,
//                           NULL,
//                           &g_left_sensor_timer);

    // PID timer
    struct repeating_timer pid_timer;
    add_repeating_timer_ms(-50, repeating_pid_handler, NULL, &pid_timer);

    TaskHandle_t h_monitor_left_sensor_task;
    xTaskCreate(monitor_left_sensor_task,
                "Monitor Left Sensor Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                READ_LEFT_SENSOR_PRIO,
                &h_monitor_left_sensor_task);

    // Left wheel
    //
    TaskHandle_t h_monitor_left_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_wheel_speed_task,
                "monitor_left_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                (void *)&g_motor_left,
                WHEEL_SPEED_PRIO,
                &h_monitor_left_wheel_speed_task_handle);

    // Right wheel
    //
    TaskHandle_t h_monitor_right_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_wheel_speed_task,
                "monitor_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                (void *)&g_motor_right,
                WHEEL_SPEED_PRIO,
                &h_monitor_right_wheel_speed_task_handle);

    // control task
    TaskHandle_t h_motor_turning_task_handle = NULL;
    xTaskCreate(motor_control_task,
                "motor_turning_task",
                configMINIMAL_STACK_SIZE,
                NULL,
                WHEEL_CONTROL_PRIO,
                &h_motor_turning_task_handle);

    vTaskStartScheduler();
}

int
main(void)
{
    stdio_usb_init();

    sleep_ms(4000);
    printf("Test started!\n");

    motor_init();

    line_sensor_setup();

    //init_ultrasonic();

    //initialize_car_state(); // TODO: Could be common functionality, To confirm
                            // during Integration
    launch();

    return (0);
}

/*** end of file ***/


#include "line_sensor.h"

#define READ_LEFT_SENSOR_PRIO           (tskIDLE_PRIORITY + 2UL)
#define READ_RIGHT_SENSOR_PRIO          (tskIDLE_PRIORITY + 2UL)

#define DIRECTION_TASK_PRIORITY         (tskIDLE_PRIORITY + 3UL)

void
launch()
{
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

    sleep_ms(2000);
    printf("Test started!\n");

    line_sensor_setup();
    initialize_car_state();

    launch();

    return (0);
}
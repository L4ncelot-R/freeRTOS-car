
#include "line_sensor_init.h"

#define READ_LEFT_SENSOR_PRIO  (tskIDLE_PRIORITY + 2UL)

void
launch(line_car_struct_t *car_struct)
{
    struct repeating_timer g_left_sensor_timer;
    add_repeating_timer_ms(LINE_SENSOR_READ_DELAY,
                           h_left_sensor_timer_handler,
                           NULL,
                           &g_left_sensor_timer);

    TaskHandle_t h_monitor_left_sensor_task = NULL;
    xTaskCreate(monitor_left_sensor_task,
                "read_left_sensor_task",
                configMINIMAL_STACK_SIZE,
                (void *)car_struct->obs,
                READ_LEFT_SENSOR_PRIO,
                &h_monitor_left_sensor_task);

    vTaskStartScheduler();
}

int
main(void)
{
    stdio_usb_init();

    obs_t obs = { 0, 0 };

    line_car_struct_t car_struct = { .obs = &obs };

    sleep_ms(2000);

    printf("Test started!\n");

    line_sensor_init(&car_struct);

    launch(&car_struct);

    return (0);
}
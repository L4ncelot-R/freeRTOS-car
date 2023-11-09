
#include "line_sensor_init.h"
#include "ultrasonic_sensor.h"
#include "car_config.h"

#define READ_LEFT_SENSOR_PRIO (tskIDLE_PRIORITY + 2UL)

void
launch(car_struct_t *car_struct)
{
    TaskHandle_t h_monitor_left_sensor_task = NULL;
    xTaskCreate(monitor_left_sensor_task,
                "read_left_sensor_task",
                configMINIMAL_STACK_SIZE,
                (void *)car_struct->obs,
                READ_LEFT_SENSOR_PRIO,
                &h_monitor_left_sensor_task);

    TaskHandle_t h_monitor_ultrasonic_task = NULL;
    xTaskCreate(check_obstacle,
                "read_ultrasonic_task",
                configMINIMAL_STACK_SIZE,
                (void *)car_struct->obs,
                READ_LEFT_SENSOR_PRIO,
                &h_monitor_ultrasonic_task);
}

int
main(void)
{
    stdio_usb_init();

    obs_t obs = { 0, 0 };

    car_struct_t car_struct = { .obs = &obs };

    sleep_ms(2000);

    printf("Test started!\n");

    line_sensor_init();
    printf("Line sensor initialized!\n");

    init_ultrasonic();
    printf("Ultrasonic sensor initialized!\n");

    launch(&car_struct);

    vTaskStartScheduler();

    return (0);
}
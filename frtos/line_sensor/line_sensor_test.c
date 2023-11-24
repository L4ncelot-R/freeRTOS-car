
#include "line_sensor_init.h"
#include "car_config.h"

int
main(void)
{
    stdio_usb_init();

    obs_t obs;

    car_struct_t car_struct = { .obs = &obs };

    sleep_ms(2000);

    printf("Test started!\n");

    line_sensor_init(&car_struct);
    printf("Line sensor initialized!\n");

    line_tasks_init(&car_struct);

    vTaskStartScheduler();

    return (0);
}
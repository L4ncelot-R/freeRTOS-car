
#include "magnetometer_init.h"
#include "magnetometer_direction.h"

int
main(void)
{
    stdio_usb_init();

    direction_t direction;

    car_struct_t car_struct = { .p_direction = &direction };

    sleep_ms(2000);
    printf("Test started!\n");

    magnetometer_init(&car_struct);

    magnetometer_tasks_init(&car_struct);

    vTaskStartScheduler();


    return (0);
}
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "ultrasonic_sensor.h"
#include "car_config.h"

int
main(void)
{
    stdio_init_all();

    obs_t obs;

    car_struct_t car_struct = { .obs = &obs };

    ultrasonic_init(&car_struct);
    sleep_ms(1000);

    ultrasonic_task_init(&car_struct);

    vTaskStartScheduler();

    return 0;
}
/*** end of file ***/
//
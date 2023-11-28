
#include "magnetometer_init.h"
#include "magnetometer_direction.h"

#define DIRECTION_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

void
launch()
{

    TaskHandle_t h_monitor_direction_task = NULL;
    xTaskCreate(monitor_direction_task,
                "Monitor Direction Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                DIRECTION_TASK_PRIORITY,
                &h_monitor_direction_task);

    vTaskStartScheduler();
}

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
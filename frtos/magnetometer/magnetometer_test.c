
#include "magnetometer_init.h"
#include "magnetometer_direction.h"
#include "map.h"

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

    int grid_rows = 10; // Define the number of rows in your grid
    int grid_cols = 10; // Define the number of columns in your grid

    car_path_grid = create_grid(grid_rows, grid_cols);

    sleep_ms(2000);
    printf("Test started!\n");

    magnetometer_init(&car_struct);

    //    printf("Magnetometer initialized!\n");

    magnetometer_tasks_init(&car_struct);

    vTaskStartScheduler();

    //    launch();

    return (0);
}
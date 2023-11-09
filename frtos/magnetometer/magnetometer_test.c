
#include "magnetometer_init.h"
#include "magnetometer_read.h"
#include "magnetometer_direction.h"
#include "map.h"

#define DIRECTION_TASK_PRIORITY             (tskIDLE_PRIORITY + 1UL)

void
launch()
{
    struct repeating_timer g_direction_timer;
    add_repeating_timer_ms(DIRECTION_READ_DELAY,
                           h_direction_timer_handler,
                           NULL,
                           &g_direction_timer);

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
main (void)
{
    stdio_usb_init();

    int grid_rows = 10;  // Define the number of rows in your grid
    int grid_cols = 10;  // Define the number of columns in your grid

    car_path_grid = create_grid(grid_rows, grid_cols);

    magnetometer_init();

    launch();

    return(0);
}

#include "mapping.h"
#include "car_config.h"

int
main(void)
{
    stdio_usb_init();
    maze_t       maze;

    sleep_ms(7000);

    printf("Test started!\n");

    mapping_init(&maze);
    printf("Mapping initialized!\n");

    mapping_tasks_init(&maze);
    printf("Mapping tasks initialized!\n");

    vTaskStartScheduler();

    return (0);
}
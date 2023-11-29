
/**
 * @file map_test.c
 * @brief   This file contains the functions to test the mapping
 *          of the car
 * @author Woon Jun Wei 2200624
 *
 * @details This file is a "Test Case" to simulate the algorithms used for
 *          mapping, and to test the mapping of the car. The Raspberry Pico
 *          will be flashed with the built binary of this file and need not
 *          be attached to the car.
 */

#include "mapping.h"
#include "car_config.h"

int
main(void)
{
    stdio_usb_init();
    maze_t maze;

    sleep_ms(3000);

    printf("Test started!\n");

    mapping_init(&maze);
    printf("Mapping initialized!\n");

    mapping_tasks_init(&maze);
    printf("Mapping tasks initialized!\n");

    vTaskStartScheduler();

    return (0);
}
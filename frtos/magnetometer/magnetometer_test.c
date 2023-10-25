
#include "magnetometer_init.h"
#include "magnetometer_read.h"
#include "magnetometer_direction.h"

//#define READ_MAGNETOMETER_PRIORITY          (tskIDLE_PRIORITY + 2UL)
//#define READ_ACCELEROMETER_PRIORITY         (tskIDLE_PRIORITY + 3UL)
#define DIRECTION_TASK_PRIORITY             (tskIDLE_PRIORITY + 1UL)

void
launch()
{
    struct repeating_timer g_direction_timer;
    add_repeating_timer_ms(1000,
                           h_direction_timer_handler,
                           NULL,
                           &g_direction_timer);

//    struct repeating_timer g_magnetometer_timer;
//    add_repeating_timer_ms(MAGNETOMETER_READ_DELAY,
//                           h_magnetometer_timer_handler,
//                           NULL,
//                           &g_magnetometer_timer);
//
//    struct repeating_timer g_accelerometer_timer;
//    add_repeating_timer_ms(ACCELEROMETER_READ_DELAY,
//                           h_accelerometer_timer_handler,
//                           NULL,
//                           &g_accelerometer_timer);

//    TaskHandle_t h_monitor_magnetometer_task = NULL;
//    xTaskCreate(monitor_magnetometer_task,
//                "Monitor Magnetometer Task",
//                configMINIMAL_STACK_SIZE,
//                NULL,
//                READ_MAGNETOMETER_PRIORITY,
//                &h_monitor_magnetometer_task);

//    TaskHandle_t h_monitor_accelerometer_task = NULL;
//    xTaskCreate(monitor_accelerometer_task,
//                "Monitor Accelerometer Task",
//                configMINIMAL_STACK_SIZE,
//                NULL,
//                READ_ACCELEROMETER_PRIORITY,
//                &h_monitor_accelerometer_task);

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

    sleep_ms(2000);

    printf("Test started!\n");
    magnetometer_init();

    launch();

    return(0);
}
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "ultrasonic_sensor.h"

void
vLaunch(void)
{   
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, echo_handler);

    irq_set_enabled(IO_IRQ_BANK0, true);

    TaskHandle_t disttask;
    xTaskCreate(distance_task,
                "TestDistThread",
                configMINIMAL_STACK_SIZE,
                NULL,
                1,
                &disttask);

    vTaskStartScheduler();
}

int
main(void)
{
    stdio_init_all();
    init_ultrasonic();
    sleep_ms(1000);
    vLaunch();

    return 0;
}
/*** end of file ***/
//
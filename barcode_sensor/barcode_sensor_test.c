#include "barcode_sensor.h"

#define READ_BARCODE_SENSOR_PRIO           (tskIDLE_PRIORITY + 2UL)

void
launch()
{
    // isr to detect left line sensor
    gpio_set_irq_enabled(BARCODE_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(BARCODE_SENSOR_PIN, h_barcode_sensor_handler);


    irq_set_enabled(IO_IRQ_BANK0, true);


   struct repeating_timer g_barcode_sensor_timer;
   add_repeating_timer_ms(LINE_SENSOR_READ_DELAY,
                          h_barcode_sensor_timer_handler,
                          NULL,
                          &g_barcode_sensor_timer);                    


    TaskHandle_t h_monitor_barcode_sensor_task;
    xTaskCreate(monitor_barcode_sensor_task,
                "Monitor Barcode Sensor Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                READ_BARCODE_SENSOR_PRIO,
                &h_monitor_barcode_sensor_task);

    vTaskStartScheduler();
}

int
main (void)
{
    stdio_usb_init();

    sleep_ms(10000);
    printf("Test started!\n");

    barcode_sensor_setup();
    initialize_car_state();

    launch();

    return (0);
}


#include "motor_speed.h"
#include "motor_direction.h"

#define READ_LEFT_WHEEL_SPEED_PRIO (tskIDLE_PRIORITY + 1UL)
#define READ_RIGHT_WHEEL_SPEED_PRIO (tskIDLE_PRIORITY + 1UL)

void
launch()
{
    // isr to detect right motor slot
    gpio_set_irq_enabled(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(SPEED_PIN_RIGHT, h_wheel_sensor_isr_handler);

    // isr to detect left motor slot
    gpio_set_irq_enabled(SPEED_PIN_LEFT, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(SPEED_PIN_LEFT, h_wheel_sensor_isr_handler);

    irq_set_enabled(IO_IRQ_BANK0, true);

    static volatile float * p_target_speed = NULL;
    static volatile float target_speed  = 30.0f; // cm/s
    p_target_speed = &target_speed;

    TaskHandle_t h_monitor_left_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_left_wheel_speed_task,
                "monitor_left_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                (void *) p_target_speed,
                READ_LEFT_WHEEL_SPEED_PRIO,
                &h_monitor_left_wheel_speed_task_handle);

    TaskHandle_t h_monitor_right_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_right_wheel_speed_task,
                "monitor_right_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                (void *) p_target_speed,
                READ_RIGHT_WHEEL_SPEED_PRIO,
                &h_monitor_right_wheel_speed_task_handle);

    vTaskStartScheduler();
}

int
main (void)
{
    stdio_usb_init();

    sleep_ms(2000);
    printf("Test started!\n");

    motor_init();
    set_wheel_direction(DIRECTION_LEFT_FORWARD | DIRECTION_RIGHT_FORWARD);

    launch();

    // for(;;);

    return (0);
}
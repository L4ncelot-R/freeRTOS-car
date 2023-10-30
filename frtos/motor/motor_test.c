

#include "motor_speed.h"
#include "motor_direction.h"
#include "motor_pid.h"

#define WHEEL_SPEED_PRIO    (tskIDLE_PRIORITY + 2UL)
#define WHEEL_CONTROL_PRIO  (tskIDLE_PRIORITY + 2UL)
#define WHEEL_PID_PRIO      (tskIDLE_PRIORITY + 2UL)

static void
motor_control_task(__unused void *p_param)
{
    for (;;)
    {
        set_wheel_direction(DIRECTION_FORWARD);
        set_wheel_speed(3000u);
        distance_to_stop(30);

        set_wheel_direction(DIRECTION_BACKWARD);
        set_wheel_speed(3000u);
        distance_to_stop(30);

        turn_wheel(DIRECTION_LEFT);
        set_wheel_direction(DIRECTION_FORWARD);
        set_wheel_speed(3000u);
        distance_to_stop(30);

        turn_wheel(DIRECTION_RIGHT);
        set_wheel_direction(DIRECTION_FORWARD);
        set_wheel_speed(3000u);
        distance_to_stop(30);

        set_wheel_speed(0u);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

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

//    set_wheel_direction(DIRECTION_FORWARD);
//    set_wheel_speed(3000);

    // Left wheel
    //
    TaskHandle_t h_monitor_left_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_wheel_speed_task,
                "monitor_left_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                (void *)&g_motor_left,
                WHEEL_SPEED_PRIO,
                &h_monitor_left_wheel_speed_task_handle);

    // Right wheel
    //
    TaskHandle_t h_monitor_right_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_wheel_speed_task,
                "monitor_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                (void *)&g_motor_right,
                WHEEL_SPEED_PRIO,
                &h_monitor_right_wheel_speed_task_handle);

    TaskHandle_t h_motor_pid_right_task_handle = NULL;
    xTaskCreate(motor_pid_task,
                "motor_pid_task",
                configMINIMAL_STACK_SIZE,
                (void *)&g_motor_right,
                WHEEL_PID_PRIO,
                &h_motor_pid_right_task_handle);

    // control task
    TaskHandle_t h_motor_turning_task_handle = NULL;
    xTaskCreate(motor_control_task,
                "motor_turning_task",
                configMINIMAL_STACK_SIZE,
                NULL,
                WHEEL_CONTROL_PRIO,
                &h_motor_turning_task_handle);

    vTaskStartScheduler();
}

int
main(void)
{
    stdio_usb_init();

    sleep_ms(4000);
    printf("Test started!\n");

    motor_init();

    launch();

    // for(;;);

    return (0);
}
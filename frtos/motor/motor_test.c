

#include "motor_speed.h"
#include "motor_direction.h"
#include "motor_pid.h"

void
motor_control_task(void *params)
{
    car_struct_t *car_struct = (car_struct_t *)params;
    for (;;)
    {
        set_wheel_direction(DIRECTION_FORWARD);
        set_wheel_speed_synced(90u, car_struct);

        vTaskDelay(pdMS_TO_TICKS(10000));

        revert_wheel_direction();
        set_wheel_speed_synced(90u, car_struct);

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void
launch(car_struct_t *car_struct, void *isr_handler)
{
    // Left wheel
    //
    TaskHandle_t h_monitor_left_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_wheel_speed_task,
                "monitor_left_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                (void *)car_struct->p_left_motor,
                WHEEL_SPEED_PRIO,
                &h_monitor_left_wheel_speed_task_handle);

    // Right wheel
    //
    TaskHandle_t h_monitor_right_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_wheel_speed_task,
                "monitor_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                (void *)car_struct->p_right_motor,
                WHEEL_SPEED_PRIO,
                &h_monitor_right_wheel_speed_task_handle);

    // control task
    TaskHandle_t h_motor_turning_task_handle = NULL;
    xTaskCreate(motor_control_task,
                "motor_turning_task",
                configMINIMAL_STACK_SIZE,
                (void *)car_struct,
                WHEEL_CONTROL_PRIO,
                &h_motor_turning_task_handle);

    // isr to detect right motor slot
    gpio_set_irq_enabled(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(SPEED_PIN_RIGHT, isr_handler);

    // isr to detect left motor slot
    gpio_set_irq_enabled(SPEED_PIN_LEFT, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(SPEED_PIN_LEFT, isr_handler);

    irq_set_enabled(IO_IRQ_BANK0, true);
}

int
main(void)
{
    stdio_usb_init();

    sleep_ms(4000);
    printf("Test started!\n");

    motor_t     g_motor_right;
    motor_t     g_motor_left;
    motor_pid_t g_pid;

    car_struct_t car_struct = { .p_right_motor = &g_motor_right,
                                .p_left_motor  = &g_motor_left,
                                .p_pid         = &g_pid };

    motor_init(&car_struct);

    launch(&car_struct, &h_wheel_sensor_isr_handler);

    // PID timer
    struct repeating_timer pid_timer;
    add_repeating_timer_ms(-50, repeating_pid_handler, &car_struct, &pid_timer);

    vTaskStartScheduler();

    return (0);
}
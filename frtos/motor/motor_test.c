
#include "motor_init.h"

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

int
main(void)
{
    stdio_usb_init();

    sleep_ms(4000);
    printf("Test started!\n");

    motor_t     motor_right;
    motor_t     motor_left;
    motor_pid_t pid;

    car_struct_t car_struct = { .p_right_motor = &motor_right,
                                .p_left_motor  = &motor_left,
                                .p_pid         = &pid };

    motor_init(&car_struct);

    motor_tasks_init(&car_struct, &h_wheel_sensor_isr_handler);

    // control task
    TaskHandle_t h_motor_turning_task_handle = NULL;
    xTaskCreate(motor_control_task,
                "motor_turning_task",
                configMINIMAL_STACK_SIZE,
                (void *)&car_struct,
                1,
                &h_motor_turning_task_handle);

    // PID timer
    struct repeating_timer pid_timer;
    add_repeating_timer_ms(-50, repeating_pid_handler, &car_struct, &pid_timer);

    vTaskStartScheduler();

    return (0);
}
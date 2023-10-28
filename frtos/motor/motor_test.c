

#include "motor_speed.h"
#include "motor_direction.h"
#include "motor_pid.h"

#define WHEEL_SPEED_PRIO (tskIDLE_PRIORITY + 1UL)

void
test_speed_change_task(void *p_param)
{
    for (;;)
    {
        g_motor_left.speed.target_cms  = 30.0f;
        g_motor_right.speed.target_cms = 30.0f;
        vTaskDelay(pdMS_TO_TICKS(5000));

        g_motor_left.speed.target_cms  = 20.0f;
        g_motor_right.speed.target_cms = 20.0f;
        vTaskDelay(pdMS_TO_TICKS(5000));

        g_motor_left.speed.target_cms  = 0.0f;
        g_motor_right.speed.target_cms = 0.0f;
        vTaskDelay(pdMS_TO_TICKS(5000));

        set_wheel_direction(DIRECTION_LEFT_BACKWARD | DIRECTION_RIGHT_BACKWARD);

        g_motor_left.speed.target_cms  = 30.0f;
        g_motor_right.speed.target_cms = 30.0f;
        vTaskDelay(pdMS_TO_TICKS(5000));

        g_motor_left.speed.target_cms  = 20.0f;
        g_motor_right.speed.target_cms = 20.0f;
        vTaskDelay(pdMS_TO_TICKS(5000));

        g_motor_left.speed.target_cms  = 0.0f;
        g_motor_right.speed.target_cms = 0.0f;
        vTaskDelay(pdMS_TO_TICKS(5000));

        set_wheel_direction(DIRECTION_LEFT_FORWARD | DIRECTION_RIGHT_FORWARD);

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

    // Left wheel
    //
    TaskHandle_t h_monitor_left_wheel_speed_task_handle = NULL;
    xTaskCreate(monitor_wheel_speed_task,
                "monitor_left_wheel_speed_task",
                configMINIMAL_STACK_SIZE,
                (void *)&g_motor_left,
                WHEEL_SPEED_PRIO,
                &h_monitor_left_wheel_speed_task_handle);

    TaskHandle_t h_motor_pid_left_task_handle = NULL;
    xTaskCreate(motor_pid_task,
                "motor_pid_task",
                configMINIMAL_STACK_SIZE,
                (void *)&g_motor_left,
                WHEEL_SPEED_PRIO,
                &h_motor_pid_left_task_handle);

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
                WHEEL_SPEED_PRIO,
                &h_motor_pid_right_task_handle);

    // Test speed change
    //
    TaskHandle_t h_test_speed_change_task_handle = NULL;
    xTaskCreate(test_speed_change_task,
                "test_speed_change_task",
                configMINIMAL_STACK_SIZE,
                NULL,
                WHEEL_SPEED_PRIO,
                &h_test_speed_change_task_handle);

    vTaskStartScheduler();
}

int
main(void)
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
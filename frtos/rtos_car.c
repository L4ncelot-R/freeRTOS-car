
#include "line_sensor_init.h"
#include "ultrasonic_sensor.h"
#include "car_config.h"
#include "motor_init.h"


/*!
 * @brief Interrupt handler for the wheel sensor
 */
void
h_isr_handler(void)
{
    if (gpio_get_irq_event_mask(SPEED_PIN_LEFT) & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(SPEED_PIN_LEFT, GPIO_IRQ_EDGE_FALL);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_left_sem,
                              &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (gpio_get_irq_event_mask(SPEED_PIN_RIGHT) & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_right_sem,
                              &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

int
main(void)
{
    stdio_usb_init();

    obs_t obs;

    motor_t     motor_right;
    motor_t     motor_left;
    motor_pid_t pid;

    direction_t direction;

    car_struct_t car_struct = { .p_right_motor = &motor_right,
                                .p_left_motor  = &motor_left,
                                .p_pid         = &pid,
                                .obs           = &obs,
                                .p_direction   = &direction };

    // Magnetometer
    magnetometer_init(&car_struct);
    printf("Magnetometer initialized!\n");

    // ultra
    ultrasonic_init(&car_struct);
    printf("Ultrasonic sensor initialized!\n");

    // line
    line_sensor_init(&car_struct);
    printf("Line sensor initialized!\n");

    // motor
    motor_init(&car_struct);
    printf("Motor initialized!\n");

    sleep_ms(1000u);

    // PID timer
    struct repeating_timer pid_timer;
    add_repeating_timer_ms(-50, repeating_pid_handler, &car_struct, &pid_timer);

    // wheel encoder
    gpio_set_irq_enabled(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(SPEED_PIN_RIGHT, h_isr_handler);

    gpio_set_irq_enabled(SPEED_PIN_LEFT, GPIO_IRQ_EDGE_FALL, true);
    gpio_add_raw_irq_handler(SPEED_PIN_LEFT, h_isr_handler);

    irq_set_enabled(IO_IRQ_BANK0, true);

    for (;;)
    {
        // motor monitoring

    }

    return (0);
}

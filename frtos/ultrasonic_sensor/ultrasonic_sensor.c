#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#define TRIG_PIN 0
#define ECHO_PIN 1

void init_ultrasonic(void)
{
    // Set up the echo pin
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    // Set up the trigger pin
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
}

void distance_task(__unused void *params)
{
    while (true)
    {
        vTaskDelay(500);

        // Trigger the ultrasonic sensor
        gpio_put(TRIG_PIN, 1);
        sleep_us(10); // Keep the trigger on for 10 microseconds
        gpio_put(TRIG_PIN, 0);

        // Wait for the echo pulse to start
        while (gpio_get(ECHO_PIN) == 0)
            tight_loop_contents();

        // Measure the pulse width (time taken for the echo to return)
        uint32_t start_time = time_us_32();
        while (gpio_get(ECHO_PIN) == 1)
            tight_loop_contents();
        uint32_t end_time = time_us_32();

        // Calculate the distance (in centimeters)
        uint32_t pulse_duration = end_time - start_time;
        float distance = pulse_duration * 0.017; // Speed of sound at ~343 m/s

        printf("Distance: %.2f cm\n", distance);

        // If gonna bang wall
        //
        if (distance < 5)
        {
            printf("Collision Imminent!\n");
            // Proc stop
            //
        }
    }
}


void vLaunch(void)
{
    TaskHandle_t disttask;
    xTaskCreate(distance_task, "TestDistThread", configMINIMAL_STACK_SIZE, NULL, 1, &disttask);

    // Start the tasks and timer running.
    //
    vTaskStartScheduler();
}

int main(void)
{
    stdio_init_all();
    init_ultrasonic();
    vLaunch();

    return 0;
}
/*** end of file ***/
//
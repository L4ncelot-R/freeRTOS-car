#include "pico/stdlib.h"

#include "frontend.h"
#include "car_config.h"

int main() {
    stdio_init_all();

    obs_t obs;
    motor_t     motor_right;
    motor_t     motor_left;
    motor_pid_t pid;

    car_struct_t car_struct = { .p_right_motor = &motor_right,
                                .p_left_motor  = &motor_left,
                                .p_pid         = &pid,
                                .obs = &obs};

    webserver_init(&car_struct);
    
    
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1);
}

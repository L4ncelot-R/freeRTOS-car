/**
 * @file ultrasonic_init.h
 * @brief define the constants and initialize the ultrasonic sensor
 * @author Poon Xiang Yuan
 */

#ifndef ULTRASONIC_INIT_H
#define ULTRASONIC_INIT_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "ultrasonic_sensor_config.h"

void
init_ultrasonic(void)
{
    // Set up the echo pin
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    // Set up the trigger pin
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
}

#endif /* ULTRASONIC_INIT_H */
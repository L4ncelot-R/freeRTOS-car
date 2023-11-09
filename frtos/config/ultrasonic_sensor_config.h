#ifndef ULTRASONIC_CONFIG_H
#define ULTRASONIC_CONFIG_H

/* ADC Configuration */

#define TRIG_PIN (2)
#define ECHO_PIN (3)

#define ULTRASONIC_SENSOR_READ_DELAY (100)

typedef struct
{
    bool obstacle_detected;
} ultrasonic_t;

#endif // ULTRASONIC_CONFIG_H

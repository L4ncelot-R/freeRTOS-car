#ifndef CONFIG_H
#define CONFIG_H

/* ADC Configuration */

#define LINE_SENSOR_READ_DELAY                  ( 100 )

#define LEFT_SENSOR_PIN                         ( 26 )
#define RIGHT_SENSOR_PIN                        ( 27 )

typedef struct s_obs_struct {
    uint8_t line_detected;
    bool ultrasonic_detected;
} obs_t;


typedef struct
{
    obs_t      *obs;

} line_car_struct_t;

#endif //CONFIG_H

#ifndef CONFIG_H
#define CONFIG_H

/* ADC Configuration */
#define VREF                                    ( 3.3f )

#define ADC_READING_DELAY_MS                    ( 300 )

#define LEFT_SENSOR_PIN                         ( 26 )
#define RIGHT_SENSOR_PIN                        ( 27 ) // Comment this line out
// if you only have one

#define THRESHOLD                               ( VREF / 2 ) // 50% of VREF

/* Map */
#define MAP_START_SYMBOL                        ( 5 )
#define MAP_SIZE 20

#endif //CONFIG_H

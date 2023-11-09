/**
 * @brief Car Init
 */

#ifndef CAR_INIT_H
#define CAR_INIT_H

#include <stdio.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "line_sensor_config.h"

struct s_obs_struct {
    bool line_detected;
    bool ultrasonic_detected;
};



#endif /* CAR_INIT_H */
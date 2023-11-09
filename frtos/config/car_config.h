#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

typedef struct s_obs_struct {
    bool line_detected;
    bool ultrasonic_detected;
} obs_t;


typedef struct
{
    obs_t      *obs;

} car_struct_t;

#endif //CAR_CONFIG_H

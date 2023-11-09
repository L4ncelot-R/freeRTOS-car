/**
 * @file ultrasonic_sensor.h
 * @brief Monitor the distance between the car and the wall
 * @author Poon Xiang Yuan
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "ultrasonic_init.h"
#include "car_config.h"

float
KalmanFilter(float U)
{
    static float R = 10;    // noise convariance can be 10, higher better smooth
    static float H = 1;     // Measurement Map scalar
    static float Q = 10;    // initial estimated convariance
    static float P = 0;     // initial error covariance
    static float U_hat = 0; // initial estimated state
    static float K     = 0; // initial Kalman gain

    // Predict
    //
    K     = P * H / (H * P * H + R);     // Update Kalman gain
    U_hat = U_hat + K * (U - H * U_hat); // Update estimated state

    // Update error covariance
    //
    P = (1 - K * H) * P + Q;

    return U_hat;
}

#endif /* ULTRASONIC_SENSOR_H */
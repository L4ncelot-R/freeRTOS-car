
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// ENA and ENB on the L298N
#define PWM_PIN_LEFT                0U // chanel A
#define PWM_PIN_RIGHT               1U // chanel B

#define DIRECTION_PIN_RIGHT_IN1     11U
#define DIRECTION_PIN_RIGHT_IN2     12U

#define DIRECTION_PIN_LEFT_IN3      19U
#define DIRECTION_PIN_LEFT_IN4      20U

#define DIRECTION_RIGHT_FORWARD     (1U << DIRECTION_PIN_RIGHT_IN2)
#define DIRECTION_RIGHT_BACKWARD    (1U << DIRECTION_PIN_RIGHT_IN1)
#define DIRECTION_LEFT_FORWARD      (1U << DIRECTION_PIN_LEFT_IN4)
#define DIRECTION_LEFT_BACKWARD     (1U << DIRECTION_PIN_LEFT_IN3)

#define SPEED_PIN_RIGHT             15U
#define SPEED_PIN_LEFT              16U

#define PWM_CLK_DIV                 250.f
#define PWM_WRAP                    5000U

#define PID_KP                      10.f
#define PID_KI                      0.0f
#define PID_KD                      0.0f

#define START_SPEED                 4900U
#define MAX_SPEED                   4900U
#define MIN_SPEED                   0U    // To be changed

#endif /* MOTOR_CONFIG_H */
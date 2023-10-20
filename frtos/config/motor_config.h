
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// ENA and ENB on the L298N
#define PWM_PIN_RIGHT               1U // chanel B
#define PWM_PIN_LEFT                0U // chanel A

// IN1, IN2, IN3, IN4 on the L298N
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

#define START_SPEED                 0U
#define MAX_SPEED                   4900U
#define MIN_SPEED                   0U    // To be changed

/*!
 * @brief Structure for the motor speed
 * @param side The side of the motor, 0 for left, 1 for right
 * @param target_speed The target speed of the motor in cm/s
 * @param pwm_level The current pwm level of the motor, in range [0, 5000]
 */
typedef struct {
    float               target_speed;
    uint16_t            pwm_level;
    SemaphoreHandle_t * sem;
    uint              * p_slice_num;
    uint                channel;
} motor_speed_t;

#endif /* MOTOR_CONFIG_H */
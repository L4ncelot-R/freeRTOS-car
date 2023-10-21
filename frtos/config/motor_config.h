
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

#define PID_KP                      3.f
#define PID_KI                      0.01f
#define PID_KD                      0.05f

#define MAX_SPEED                   4900U
#define MIN_SPEED                   0U    // To be changed

/*!
 * @brief Structure for the motor speed
 * @param target_speed The target speed of the wheel, in cm/s
 * @param pwm_level The pwm level of the wheel, from 0 to 5000
 * @param sem The semaphore for the wheel
 * @param p_slice_num The pointer to the slice number of the wheel
 * @param channel The pwm channel of the wheel, left A or right B
 */
typedef struct {
    float               target_speed_cms;
    uint16_t            pwm_level;
    SemaphoreHandle_t * p_sem;
    uint              * p_slice_num;
    uint                pwm_channel;
} motor_speed_t;

#endif /* MOTOR_CONFIG_H */
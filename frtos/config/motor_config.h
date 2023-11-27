
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// ENA and ENB on the L298N
#define PWM_PIN_RIGHT 1U // chanel B
#define PWM_PIN_LEFT  0U // chanel A

#define SLICE_NUM     0U

// IN1, IN2, IN3, IN4 on the L298N
#define DIRECTION_PIN_RIGHT_IN1 7U //11
#define DIRECTION_PIN_RIGHT_IN2 6U //12

#define DIRECTION_PIN_LEFT_IN3 3U //19
#define DIRECTION_PIN_LEFT_IN4 2U //20

// to turn one side
#define DIRECTION_RIGHT_FORWARD  (1U << DIRECTION_PIN_RIGHT_IN2)
#define DIRECTION_RIGHT_BACKWARD (1U << DIRECTION_PIN_RIGHT_IN1)
#define DIRECTION_LEFT_FORWARD   (1U << DIRECTION_PIN_LEFT_IN4)
#define DIRECTION_LEFT_BACKWARD  (1U << DIRECTION_PIN_LEFT_IN3)

// to spin
#define DIRECTION_FORWARD  (DIRECTION_LEFT_FORWARD | DIRECTION_RIGHT_FORWARD)
#define DIRECTION_BACKWARD (DIRECTION_LEFT_BACKWARD | DIRECTION_RIGHT_BACKWARD)
#define DIRECTION_LEFT     (DIRECTION_LEFT_BACKWARD | DIRECTION_RIGHT_FORWARD)
#define DIRECTION_RIGHT    (DIRECTION_LEFT_FORWARD | DIRECTION_RIGHT_BACKWARD)

#define DIRECTION_MASK (DIRECTION_FORWARD | DIRECTION_BACKWARD)

// wheel encoder sensor pins
#define SPEED_PIN_RIGHT 15U
#define SPEED_PIN_LEFT  16U

// PWM parameters
#define PWM_CLK_DIV 50.f
#define PWM_WRAP    100U

#define MAX_PWM_LEVEL 99U
#define MIN_PWM_LEVEL 0U

// speed in cm/s; speed = distance / time
// distance = circumference / 20
// circumference = 2 * pi * 3.25 cm = 20.4203522483 cm
// distance = 20.4203522483 cm / 20 = 1.02101761242 cm
#define SLOT_DISTANCE_CM          1.02101761242f
#define SLOT_DISTANCE_CM_MODIFIED (SLOT_DISTANCE_CM * 1000000.f)

/*!
 * @brief Structure for the motor speed parameters
 * @param current_speed_cms Current speed in cm/s
 * @param distance_cm Distance travelled in cm
 */
typedef struct
{
    float current_cms;
    float distance_cm;

} motor_speed_t;

/*!
 * @brief Structure for the motor PWM parameters
 * @param slice_num PWM slice number
 * @param pwm_channel PWM channel, either A or B
 * @param pwm_level PWM level, from 0 to 5000
 */
typedef struct
{
    uint     channel;
    uint16_t level;
} motor_pwm_t;

/*!
 * @brief Structure for the motor PID parameters
 * @param pid_kp Proportional gain
 * @param pid_ki Integral gain
 * @param pid_kd Derivative gain
 * @param use_pid Flag to use PID or not
 */
typedef struct
{
    bool  use_pid;
    float kp_value;
    float ki_value;
    float kd_value;
} motor_pid_t;

/*!
 * @brief Structure for the motor parameters
 * @param speed Motor speed parameters
 * @param pwm Motor PWM parameters
 * @param p_sem Pointer to the semaphore
 * @param use_pid Pointer to the use_pid flag
 */
typedef struct
{
    motor_speed_t      speed;
    motor_pwm_t        pwm;
    SemaphoreHandle_t *p_sem;
    bool              *use_pid;

} motor_t;

SemaphoreHandle_t g_left_sem;
SemaphoreHandle_t g_right_sem;


#endif /* MOTOR_CONFIG_H */
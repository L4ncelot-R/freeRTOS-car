
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// ENA and ENB on the L298N
#define PWM_PIN_RIGHT 1U // chanel B
#define PWM_PIN_LEFT  0U // chanel A

// IN1, IN2, IN3, IN4 on the L298N
#define DIRECTION_PIN_RIGHT_IN1 11U
#define DIRECTION_PIN_RIGHT_IN2 12U

#define DIRECTION_PIN_LEFT_IN3 19U
#define DIRECTION_PIN_LEFT_IN4 20U

#define DIRECTION_RIGHT_FORWARD     (1U << DIRECTION_PIN_RIGHT_IN2)
#define DIRECTION_RIGHT_BACKWARD    (1U << DIRECTION_PIN_RIGHT_IN1)
#define DIRECTION_LEFT_FORWARD      (1U << DIRECTION_PIN_LEFT_IN4)
#define DIRECTION_LEFT_BACKWARD     (1U << DIRECTION_PIN_LEFT_IN3)

#define DIRECTION_FORWARD   (DIRECTION_LEFT_FORWARD | DIRECTION_RIGHT_FORWARD)
#define DIRECTION_BACKWARD  (DIRECTION_LEFT_BACKWARD | DIRECTION_RIGHT_BACKWARD)
#define DIRECTION_LEFT      (DIRECTION_LEFT_BACKWARD | DIRECTION_RIGHT_FORWARD)
#define DIRECTION_RIGHT     (DIRECTION_LEFT_FORWARD | DIRECTION_RIGHT_BACKWARD)

#define DIRECTION_MASK  (DIRECTION_FORWARD | DIRECTION_BACKWARD)

#define SPEED_PIN_RIGHT 15U
#define SPEED_PIN_LEFT  16U

#define PWM_CLK_DIV 50.f
#define PWM_WRAP    100U

#define MAX_PWM_LEVEL 99U
#define MIN_PWM_LEVEL 0U

#define WHEEL_SPEED_PRIO    (tskIDLE_PRIORITY + 1UL)
#define WHEEL_CONTROL_PRIO  (tskIDLE_PRIORITY + 1UL)
#define WHEEL_PID_PRIO      (tskIDLE_PRIORITY + 1UL)

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
    uint     slice_num;
    uint     channel;
    uint16_t level;
} motor_pwm_t;

/*!
 * @brief Structure for the motor PID parameters
 * @param pid_kp Proportional gain
 * @param pid_ki Integral gain
 * @param pid_kd Derivative gain
 */
typedef struct
{
    float kp_value;
    float ki_value;
    float kd_value;
} motor_pid_t;

/*!
 * @brief Structure for the motor parameters
 * @param speed Motor speed parameters
 * @param sem Semaphore for the motor speed
 * @param pwm Motor PWM parameters
 * @param pid Motor PID parameters
 */
typedef struct
{
    motor_speed_t     speed;
    SemaphoreHandle_t sem;
    motor_pwm_t       pwm;
    motor_pid_t       pid;
} motor_t;

typedef struct
{
    float         starting_distance_cm;
    float         distance_to_travel_cm;
    volatile bool is_running;
} distance_to_stop_t;

volatile bool g_use_pid = true;

#endif /* MOTOR_CONFIG_H */
// GPIO 2 as the PWM output, GPIO 26 as the ADC input
#define PWM_PIN_LEFT  0U // chanel A
#define PWM_PIN_RIGHT 1U // chanel B

#define DIRECTION_PIN_RIGHT_IN1 11U
#define DIRECTION_PIN_RIGHT_IN2 12U

#define DIRECTION_PIN_LEFT_IN3 19U
#define DIRECTION_PIN_LEFT_IN4 20U

#define DIRECTION_RIGHT_FORWARD   (1U << DIRECTION_PIN_RIGHT_IN2)
#define DIRECTION_RIGHT_BACKWARD  (1U << DIRECTION_PIN_RIGHT_IN1)
#define DIRECTION_LEFT_FORWARD    (1U << DIRECTION_PIN_LEFT_IN4)
#define DIRECTION_LEFT_BACKWARD   (1U << DIRECTION_PIN_LEFT_IN3)

#define SPEED_PIN_RIGHT 15U
#define SPEED_PIN_LEFT  16U

#define PWM_CLK_DIV 250.f
#define PWM_WRAP    5000U

#define PID_KP 10.f
#define PID_KI 0.0f
#define PID_KD 0.0f

#define START_SPEED         1500U
#define MAX_SPEED           4900U
#define MIN_SPEED           0U    // To be changed

uint g_slice_num_left  = 0U;
uint g_slice_num_right = 0U;

SemaphoreHandle_t g_wheel_speed_sem_left = NULL;
SemaphoreHandle_t g_wheel_speed_sem_right = NULL;

void
wheel_setup(void)
{
  // Semaphore
  g_wheel_speed_sem_left = xSemaphoreCreateBinary();
  g_wheel_speed_sem_right = xSemaphoreCreateBinary();

  gpio_init(SPEED_PIN_RIGHT);
  gpio_init(SPEED_PIN_LEFT);
  gpio_set_dir(SPEED_PIN_RIGHT, GPIO_IN);
  gpio_set_dir(SPEED_PIN_LEFT, GPIO_IN);

  // Initialize direction pins as outputs
  gpio_init(DIRECTION_PIN_RIGHT_IN1);
  gpio_init(DIRECTION_PIN_RIGHT_IN2);
  gpio_init(DIRECTION_PIN_LEFT_IN3);
  gpio_init(DIRECTION_PIN_LEFT_IN4);

  gpio_set_dir(DIRECTION_PIN_RIGHT_IN1, GPIO_OUT);
  gpio_set_dir(DIRECTION_PIN_RIGHT_IN2, GPIO_OUT);
  gpio_set_dir(DIRECTION_PIN_LEFT_IN3, GPIO_OUT);
  gpio_set_dir(DIRECTION_PIN_LEFT_IN4, GPIO_OUT);

  // Initialise PWM
  gpio_set_function(PWM_PIN_LEFT, GPIO_FUNC_PWM);
  gpio_set_function(PWM_PIN_RIGHT, GPIO_FUNC_PWM);

  g_slice_num_left = pwm_gpio_to_slice_num(PWM_PIN_LEFT);
  g_slice_num_right = pwm_gpio_to_slice_num(PWM_PIN_RIGHT);

  // NOTE: PWM clock is 125MHz for raspberrypi pico w by default

  // 125MHz / 250 = 500kHz
  pwm_set_clkdiv(g_slice_num_left, PWM_CLK_DIV);
  pwm_set_clkdiv(g_slice_num_right, PWM_CLK_DIV);

  // have them to be 500kHz / 5000 = 100Hz
  pwm_set_wrap(g_slice_num_left,  (PWM_WRAP - 1U));
  pwm_set_wrap(g_slice_num_right, (PWM_WRAP - 1U));

  pwm_set_enabled(g_slice_num_left, true);
  pwm_set_enabled(g_slice_num_right, true);
}


/*!
* @brief Set the direction of the wheels; can use bitwise OR to set both
* wheels such as DIRECTION_LEFT_FORWARD | DIRECTION_RIGHT_BACKWARD, it will
* set the left wheel to go forward and the right wheel to go backward within
* the same function.
* if the wheel direction is not set, it will not move.
* @param direction The direction of the left and right wheels
* @param left_speed The speed of the left wheel, from 0.0 to 1.0
* @param right_speed The speed of the right wheel, from 0.0 to 1.0
 */
void
set_wheel_direction (uint32_t direction)
{
    static const uint32_t mask = DIRECTION_LEFT_FORWARD |
                                 DIRECTION_LEFT_BACKWARD |
                                 DIRECTION_RIGHT_FORWARD |
                                 DIRECTION_RIGHT_BACKWARD;

    gpio_put_masked(mask, 0U);
    gpio_set_mask(direction);
}

/*!
 * @brief Set the speed of the wheels; can use bitwise OR to set both
 * @param speed in range [0, 5000]
 * @param side 0 for left, 1 for right
 */
void
set_wheel_speed (float speed, uint8_t side)
{
    if (side == 0U)
    {
        pwm_set_chan_level(g_slice_num_left,
                         PWM_CHAN_A,
                         (uint16_t) speed);
    }
    else
    {
        pwm_set_chan_level(g_slice_num_right,
                           PWM_CHAN_B,
                           (uint16_t) speed);
    }

}

void
h_left_wheel_sensor_isr_handler (void)
{
    if (gpio_get_irq_event_mask(SPEED_PIN_LEFT) & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(SPEED_PIN_LEFT, GPIO_IRQ_EDGE_FALL);

        // printf("left wheel sensor isr\n");
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_wheel_speed_sem_left, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void
h_right_wheel_sensor_isr_handler (void)
{
    if (gpio_get_irq_event_mask(SPEED_PIN_RIGHT) & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(SPEED_PIN_RIGHT, GPIO_IRQ_EDGE_FALL);

        // printf("right wheel sensor isr\n");
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_wheel_speed_sem_right, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

float
compute_pid(float target_speed, float current_speed, float * integral, float * prev_error)
{
    float error = target_speed - current_speed;
    *integral += error;

    float derivative = error - *prev_error;

    float control_signal = PID_KP * error +
                           PID_KI * (*integral) +
                           PID_KD * derivative;

    *prev_error = error;

    return control_signal;
}

void
monitor_left_wheel_speed_task (void *pvParameters)
{
    static float * target_speed = NULL;
    *target_speed = * (float *) pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(g_wheel_speed_sem_left, portMAX_DELAY) == pdTRUE)
        {
            static uint64_t curr_time_left  = 0u;
                            curr_time_left  = time_us_64();


            static uint64_t prev_time_left    = 0u;
            static uint64_t elapsed_time_left = 1u; // to avoid division by 0

            elapsed_time_left = curr_time_left - prev_time_left;

            prev_time_left = curr_time_left;

            static float speed_left = 0.f;
            // speed in cm/s; speed = distance / time
            // distance = circumference / 20
            // circumference = 2 * pi * 3.25 cm = 20.4203522483 cm
            // distance = 20.4203522483 cm / 20 = 1.02101761242 cm
            speed_left = (float)
                (1.02101761242f / (elapsed_time_left / 1000000.f));

            printf("left speed: %f cm/s\n", speed_left);

        }

    }

}

void
monitor_right_wheel_speed_task (void *pvParameters)
{
    // volatile float * target_speed = (float *) pvParameters;
    static volatile float * target_speed = NULL;
    target_speed = (float *) pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(g_wheel_speed_sem_right, portMAX_DELAY) == pdTRUE)
        {
            static uint64_t curr_time_right  = 0u;
                            curr_time_right  = time_us_64();

            static uint64_t prev_time_right    = 0u;
            static uint64_t elapsed_time_right = 1u; // to avoid division by 0

            elapsed_time_right = curr_time_right - prev_time_right;

            prev_time_right = curr_time_right;

            static float speed_right = 0.f;

            speed_right = (float)
                (1.02101761242f / (elapsed_time_right / 1000000.f));

            printf("right speed: %f cm/s\n", speed_right);

            static float control_signal = 0.f;
            static float integral = 0.f;
            static float prev_error = 0.f;

            control_signal = compute_pid(*target_speed,
                                         speed_right,
                                         &integral,
                                         &prev_error);

            static float new_pwm = START_SPEED;

            if (new_pwm + control_signal > MAX_SPEED)
            {
                new_pwm = MAX_SPEED;
            }
            else if (new_pwm + control_signal < MIN_SPEED)
            {
                new_pwm = MIN_SPEED;
            }
            else
            {
                new_pwm = new_pwm + control_signal;
            }

            printf("control signal: %f\n", control_signal);
            printf("new pwm: %f\n\n", new_pwm);

            set_wheel_speed(new_pwm, 1u);

        }

    }

}
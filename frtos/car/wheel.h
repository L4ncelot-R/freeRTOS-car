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

#define SPEED_PIN_RIGHT 27U // ADC0
#define SPEED_PIN_LEFT  26U // ADC1

#define PWM_CLK_DIV 250.f
#define PWM_WRAP    5000U

#define ADC_READING_TRESHOLD 3500u

#define SPEED_READING_TRESHOLD_MSEC 1000u

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

  // Speed
  /*    adc_init();
      adc_gpio_init(SPEED_PIN_RIGHT);
      adc_gpio_init(SPEED_PIN_LEFT);*/

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
 * @param speed in range [0.0, 1.0]
 * @param side 0 for left, 1 for right
 */
void
set_wheel_speed (float speed, uint8_t side)
{
    if (side == 0U)
    {
        pwm_set_chan_level(g_slice_num_left,
                           PWM_CHAN_A,
                           (short) (PWM_WRAP * speed));
    }
    else
    {
        pwm_set_chan_level(g_slice_num_right,
                           PWM_CHAN_B,
                           (short) (PWM_WRAP * speed));
    }

}

void
left_wheel_sensor_isr (__unused uint gpio, __unused uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_wheel_speed_sem_left, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void
right_wheel_sensor_isr (__unused uint gpio, __unused uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_wheel_speed_sem_right, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void
monitor_left_wheel_speed_task (__unused void *pvParameters)
{
    for (;;)
    {
        if (xSemaphoreTake(g_wheel_speed_sem_left, portMAX_DELAY) == pdTRUE)
        {
            static uint64_t curr_time_left  = 0u;
                            curr_time_left  = time_us_64();


            static uint64_t prev_time_left    = 0u;
            static uint64_t elapsed_time_left = 0u;

            elapsed_time_left = curr_time_left - prev_time_left;

            printf("time elapsed: %llu\n", elapsed_time_left);

            prev_time_left = curr_time_left;

        }

    }

}

void
monitor_right_wheel_speed_task (__unused void *pvParameters)
{
    for (;;)
    {
        if (xSemaphoreTake(g_wheel_speed_sem_right, portMAX_DELAY) == pdTRUE)
        {
            static uint64_t curr_time_right  = 0u;
                            curr_time_right  = time_us_64();

            static uint64_t prev_time_right    = 0u;
            static uint64_t elapsed_time_right = 0u;

            elapsed_time_right = curr_time_right - prev_time_right;

            printf("time elapsed: %llu\n", elapsed_time_right);

            prev_time_right = curr_time_right;
        }

    }

}
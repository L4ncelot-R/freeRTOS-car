#include "line_sensor.h"
#include "string.h"


const float conversionFactor = 3.3f / (1 << 12);

volatile u_int8_t map[MAP_SIZE][MAP_SIZE] = {0};

/**
 * @brief Get the status of the Line Sensor
 * @param gpio The GPIO pin to read from
 * @return state_t LINE_DETECTED or LINE_NOT_DETECTED
 */
static inline state_t
get_state(uint gpio) {
    adc_select_input(gpio - 26); // 26 is the first ADC pin (ADC0)

    // ADC Conversion, vref is 3.3V, 12 bit resolution
    float adc = (float) adc_read() * conversionFactor;

    return (adc > THRESHOLD)
           ? LINE_DETECTED : LINE_NOT_DETECTED;
}

/**
 * @brief Get the Direction based on the Line Sensor
 *
 * This function will detect if the line sensor is detecting a line or not
 * It will read from both ADC channels and return if both are detecting a line
 *
 * @return state_t LINE_DETECTED or LINE_NOT_DETECTED
 */
static inline
direction_t get_current_dir() {
#ifdef ADC_PIN_TWO
    state_t state_two = get_state(ADC_PIN_TWO);
#endif
    state_t state_one = get_state(ADC_PIN_ONE);

#ifdef ADC_PIN_TWO
    direction_t direction = (state_one << 1) | state_two;
#else
    direction_t direction = state_one;
#endif

    return direction;
}


/**
 * @brief Setup the Line Sensor
 *
 * This function will setup the Line Sensor by initializing the ADC
 */
void
line_sensor_setup() {

    adc_init();
    adc_gpio_init(ADC_PIN_ONE);
#ifdef ADC_PIN_TWO
    adc_gpio_init(ADC_PIN_TWO);
#endif

#ifdef ADC_PIN_THREE            // Additional ADC Pin for Barcode Scanner
    adc_gpio_init(ADC_PIN_THREE);
#endif

}

/**
 * @brief Initialize the car's initial state
 *
 * @return The initialized car state
 */
car_state_t
initialize_car_state() {
    car_state_t car_state;
    car_state.x = MAP_SIZE >> 1;
    car_state.y = MAP_SIZE >> 1;
    car_state.current_direction = FORWARD;
    car_state.orientation = NORTH;

    map[car_state.x][car_state.y] = MAP_START_SYMBOL;

    return car_state;
}

/**
 * @brief Update the map based on the car's state
 *
 * @param car_state The current car state
 */
static inline void
update_map(car_state_t car_state) {
    if (car_state.x >= 0 && car_state.x < MAP_SIZE &&
        car_state.y >= 0 && car_state.y < MAP_SIZE) {
        map[car_state.x][car_state.y] = 1;
    }
}

/**
 * @brief Handle forward movement of the car
 *
 * @param car_state The current car state
 */
static void
handle_forward_movement(car_state_t *car_state) {
    printf("FORWARD, ");
    // TODO: Check car's actual forward movement
    switch (car_state->orientation) {
        case NORTH:
            printf("NORTH\n");
            car_state->y++;
            break;
        case EAST:
            printf("EAST\n");
            car_state->x++;
            break;
        case SOUTH:
            printf("SOUTH\n");
            car_state->y--;
            break;
        case WEST:
            printf("WEST\n");
            car_state->x--;
            break;
    }
}

/**
 * @brief Handle a right turn of the car
 *
 * Note: Bitwise AND with 0x03 to ensure that the orientation
 * is always between 0 and 3
 * @param car_state The current car state
 */
static inline void
handle_right_turn(car_state_t *car_state) {
    car_state->orientation = (car_state->orientation + 1) & 0x03;
}

/**
 * @brief Handle a left turn of the car
 *
 * @param car_state The current car state
 */
static inline void
handle_left_turn(car_state_t *car_state) {
    car_state->orientation = (car_state->orientation - 1) & 0x03;
}

/**
 * @brief Print the map to the console
 *
 * This function will print the map to the console
 */
void
print_map() {
    // Invert the map, 0,0 is at the bottom left
    for (int i = MAP_SIZE - 1; i >= 0; i --)
    {
        for (int j = 0; j < MAP_SIZE; j ++)
        {
            printf("%d ", map[j][i]);
        }
        printf("\n");
    }
}

/**
 * @brief FreeRTOS Task to read from the line sensor
 *
 * This task will read from the line sensor and print the state to the console
 *
 * @param params Pointer to the task parameters
 */
void
line_sensor_task(__unused void *params) {

    car_state_t car_state = initialize_car_state();

    sleep_ms(2000);
    printf("Starting Line Sensor Task\n");


    for (u_int8_t i = 40; i != 0; i--) {
        car_state.current_direction = get_current_dir();

        switch (car_state.current_direction) {
            case FORWARD:
                handle_forward_movement(&car_state);
                break;
            case RIGHT:
                handle_right_turn(&car_state);
                break;
            case LEFT:
                handle_left_turn(&car_state);
                break;
            default:
                break;
        }

        update_map(car_state);
        sleep_ms(ADC_READING_DELAY_MS);
    }

    print_map();

}
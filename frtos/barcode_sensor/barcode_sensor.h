/* Barcode sensor */

#include "barcode_sensor_init.h"

#define MAX_BARCODES 10 // Define the maximum number of barcodes to store
#define BARCODE_SENSOR_TIMER_PERIOD_MS 100 // Define the barcode sensor timer period

// Define the barcode sensor timer
static struct repeating_timer barcode_sensor_timer;


/**
 * @brief Decode a Code 39 barcode
 *
 * This function decodes a Code 39 barcode represented as a 9-bit binary number.
 *
 * @param barcode_data Binary representation of the barcode data (9 bits)
 * @return Decoded value as an integer
 */
int code39_decode(uint32_t barcode_data) {
    // Define the binary representations of Code 39 characters
    const uint32_t code39_characters[] = {
        0b001001001,  // 0
        0b001001011,  // 1
        0b001011001,  // 2
        0b001011011,  // 3
        0b001100011,  // 4
        0b001101001,  // 5
        0b001101011,  // 6
        0b001010011,  // 7
        0b001011101,  // 8
        0b001111001,  // 9
        // Add more character representations as needed
    };

    // Compare the barcode data to known Code 39 character representations
    for (int i = 0; i < 10; i++) {
        if (barcode_data == code39_characters[i]) {
            return i;  // Return the decoded value (0-9)
        }
    }

    // If the barcode data does not match any known character, return -1 to indicate an error
    return -1;
}


/**
 * @brief Monitor the barcode sensor
 *
 * This function will monitor the barcode sensor and send the state to the
 * barcode sensor message buffer, including Code 39 decoding.
 *
 * @param params
 */
void monitor_barcode_sensor_task(void *params) {
    // Create the barcode sensor timer
    add_repeating_timer_ms(BARCODE_SENSOR_TIMER_PERIOD_MS, h_barcode_sensor_timer_handler, NULL, &barcode_sensor_timer);

    for (;;) {
        if (xSemaphoreTake(g_barcode_sensor_sem, portMAX_DELAY) == pdTRUE) {
            // Check the flag or receive the message
            if (barcode_sensor_triggered == pdTRUE) {
                uint32_t barcode_data = 0;
                int bar_width = 0; // Variable to store the width of the current bar

                for (int i = 0; i < 9; i++) {
                    sleep_ms(100);  // Wait for a segment of the barcode

                    // Measure bar width using the IR sensor
                    if (gpio_get(BARCODE_SENSOR_PIN)) {
                        bar_width++;
                    } else {
                        // Bar ended, process the width
                        if (bar_width > 0) {
                            printf("Bar Width: %d\n", bar_width);
                            // Process or store the bar width as needed
                            bar_width = 0; // Reset the bar width measurement
                        }

                        barcode_data |= (1u << i);
                    }
                }

                printf("Barcode Data (binary): %09b\n", barcode_data);

                // Decode the barcode data
                int decoded_value = code39_decode(barcode_data);

                if (decoded_value != -1) {
                    printf("Decoded Value: %d\n", decoded_value);
                    // Store or process the decoded value as needed

                    // Send the decoded value instead of the raw barcode data
                    xMessageBufferSend(barcode_sensor_msg_buffer, &decoded_value, sizeof(int), 0);
                } else {
                    printf("Error: Unable to decode the barcode.\n");
                }

                // Reset the flag
                barcode_sensor_triggered = pdFALSE;
            }
        }
    }
}




/**
 * @brief Monitor the barcode sensor
 * @param params
 */
void
monitor_barcode_task(__unused void *params) {
    state_t barcode_state;

        // Receive from Buffer
        xMessageBufferReceive(barcode_sensor_msg_buffer,
                              &barcode_state,
                              sizeof(state_t),
                              portMAX_DELAY);
        
}
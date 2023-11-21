/* Barcode sensor */

#include "barcode_sensor_init.h"
#include <string.h>

#define MAX_BARCODES 10 // Define the maximum number of barcodes to store
#define BARCODE_SENSOR_TIMER_PERIOD_MS 100 // Define the barcode sensor timer period

// Define the barcode sensor timer
static struct repeating_timer barcode_sensor_timer;

// Define the barcode data array
static uint32_t barcode_data_array[MAX_BARCODES];
static int barcode_data_index = 0;

/**
 * @brief Decode a Code 39 barcode
 *
 * This function decodes a Code 39 barcode represented as a 9-bit binary number.
 *
 * @param barcode_data Binary representation of the barcode data (9 bits)
 * @return Decoded value as a character
 */
char code39_decode(uint32_t barcode_data) {
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
        0b001010101,  // A
        0b001010111,  // B
        0b001011101,  // C
        0b001101101,  // D
        0b001110101,  // E
        0b001110111,  // F
        0b001111101,  // G
        0b001101011,  // H
        0b001011011,  // I
        0b001010011,  // J
        0b001101111,  // K
        0b001111011,  // L
        0b001110011,  // M
        0b001110001,  // N
        0b001101001,  // O
        0b001011001,  // P
        0b001010001,  // Q
        0b001001101,  // R
        0b001001111,  // S
        0b001001011,  // T
        0b001111011,  // U
        0b001111101,  // V
        0b001110101,  // W
        0b001010111,  // X
        0b001011111,  // Y
        0b001101101,  // Z
        0b001110011,  // -
        0b001011001,  // .
        0b001101111,  // space
        0b001111111,  // $
        0b001010011,  // /
        0b001010001,  // +
        0b001001001,  // %
    };

    // Compare the barcode data to known Code 39 character representations
    for (int i = 0; i < 44; i++) {
        if (barcode_data == code39_characters[i]) {
            return (char)(i + 48);  // Return the decoded value as a character
        }
    }

    // If the barcode data does not match any known character, return '?' to indicate an error
    return '?';
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

                        barcode_data_array[barcode_data_index] |= (1u << i);
                    }
                }

                printf("Barcode Data (binary): %09b\n", barcode_data_array[barcode_data_index]);

                barcode_data_index++;

                if (barcode_data_index >= MAX_BARCODES) {
                    // Decode the barcode data
                    for (int i = 0; i < MAX_BARCODES; i++) {
                        int ones_count = 0;
                        uint32_t barcode_data = barcode_data_array[i];

                        // Count the number of ones in the barcode data
                        for (int j = 0; j < 9; j++) {
                            if ((barcode_data >> j) & 1) {
                                ones_count++;
                            }
                        }

                        char decoded_value = code39_decode(barcode_data);

                        printf("Decoded Value: %c\n", decoded_value);
                        // Store or process the decoded value as needed

                        // Send the decoded value instead of the raw barcode data
                        xMessageBufferSend(barcode_sensor_msg_buffer, &decoded_value, sizeof(char), 0);
                    }

                    // Reset the barcode data array and index
                    memset(barcode_data_array, 0, sizeof(barcode_data_array));
                    barcode_data_index = 0;
                }

                // Reset the flag
                barcode_sensor_triggered = pdFALSE;
            }
        }
    }
}

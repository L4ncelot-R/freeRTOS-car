/* Initialise the barcode sensor */

#ifndef BARCODE_SENSOR_INIT_H
#define BARCODE_SENSOR_INIT_H
#define DEBOUNCE_DELAY_MS 100

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "semphr.h"

#include "line_sensor_config.h"
#include "line_sensor_init.h"


// Set barcode time to 0
static TickType_t lastBarcodeTime = 0;


// Semaphore
SemaphoreHandle_t g_barcode_sensor_sem = NULL;


// Queue
static MessageBufferHandle_t barcode_sensor_msg_buffer;   // Barcode Sensor Buffer

// Flag
static volatile BaseType_t barcode_sensor_triggered = pdFALSE;


/**
 * @brief Setup the Line Sensor
 *
 * This function will setup the Line Sensor by initializing it as an input
 */
static inline void
barcode_sensor_setup() {
    g_barcode_sensor_sem = xSemaphoreCreateBinary();


    uint mask = (1 << BARCODE_SENSOR_PIN);

    // Initialise 3 GPIO pins and set them to input
    gpio_init_mask(mask);
    gpio_set_dir_in_masked(mask);

    barcode_sensor_msg_buffer = xMessageBufferCreate(30);

}

/**
 * @brief Timer Interrupt Handler for the barcode sensor
 *
 * @param repeatingTimer
 * @return True (To keep the timer running)
 */
bool h_barcode_sensor_timer_handler(repeating_timer_t *repeatingTimer) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_barcode_sensor_sem,
                          &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return true;
}

void h_barcode_sensor_handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TickType_t currentTicks = xTaskGetTickCount();
    printf("Interrupt triggered\n");

    if (gpio_get_irq_event_mask(BARCODE_SENSOR_PIN) & GPIO_IRQ_EDGE_FALL)
    {
        if ((currentTicks - lastBarcodeTime) >=
            pdMS_TO_TICKS(DEBOUNCE_DELAY_MS))
        {
            lastBarcodeTime = currentTicks;
            gpio_acknowledge_irq(BARCODE_SENSOR_PIN, GPIO_IRQ_EDGE_FALL);
            // Set the flag to notify the task
            barcode_sensor_triggered = pdTRUE;
            xSemaphoreGiveFromISR(g_barcode_sensor_sem,
                                  &xHigherPriorityTaskWoken);
        }
        else
        {
            // Reset the timer to the currentTicks if the edge is ignored
            lastBarcodeTime = currentTicks;
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#endif /* LINE_SENSOR_INIT_H */

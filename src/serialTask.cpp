// === INCLUDE === //

#include "tasks.h"
#include "config.h"

// === GLOBAL VARIABLES === //

// Task handles
TaskHandle_t serialTaskHandle = nullptr;

// === EXTERNALS === //

// === TASK === //

void serialTask(void *pvParameters) {
    (void)pvParameters;

    /* Make the task execute at a specified frequency */
    const TickType_t xFrequency = configTICK_RATE_HZ / SERIAL_TASK_FREQUENCY;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("Set up serialTask");

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        Serial.println("This is the serial task");

    }
}
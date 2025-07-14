// === INCLUDE === //

#include "tasks.h"
#include "config.h"

// === GLOBAL VARIABLES === //

// Task handles
TaskHandle_t controlTaskHandle = nullptr;

// === EXTERNALS === //

// === TASK === //

void controlTask(void *pvParameters) {
    (void)pvParameters;

    /* Make the task execute at a specified frequency */
    const TickType_t xFrequency = configTICK_RATE_HZ / CONTROL_TASK_FREQUENCY;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("Set up controlTask");

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        Serial.println("This is the control task");

    }
}
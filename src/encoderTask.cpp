// === INCLUDE === //

#include "tasks.h"
#include "config.h"

#include "AS5600.h"

// === GLOBAL VARIABLES === //

// Task handles
TaskHandle_t encoderTaskHandle = nullptr;

std::array<AS5600, N_ENCODERS> encoderArray;
std::array<float, N_ENCODERS> encoderPositionArray;

// === EXTERNALS === //
extern std::array<float, N_ENCODERS> currentAngleArray;

// === INTERRUPT === //

void IRAM_ATTR sampleEncodersISR() {
    // TODO: [SAM] Sample encoders here and write to encoderPositionArray
    encoderPositionArray[0] = encoderPositionArray[0] + 1; //replace this
}

// === TASK === //

void encoderTask(void *pvParameters) {
    (void)pvParameters;

    // Set up encoders
    //TODO: [SAM] Use encoderArray to store 4 initialised as5600 objects
    // Note; when initialising, make sure to use correct AS5600 object (can choose AS5600 or AS5600L)

    /* Make the task execute at a specified frequency */
    const TickType_t xFrequency = configTICK_RATE_HZ / ENCODER_TASK_FREQUENCY;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("Set up encoderTask");

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        //TODO: Perform conversion from encoderPositions to motorPositions and currentAngles
        Serial.println(encoderPositionArray[0]); //use for debugging, since can't print from ISR
    }
}
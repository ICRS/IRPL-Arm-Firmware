// === INCLUDE === //

#include "tasks.h"
#include "config.h"

#include "freertos/task.h"

// === SETUP === //

void setup() {

  Serial.begin(115200);
  Serial.println("Hello World!");

    // === CREATE TASKS === //
#if DEBUG_TASK_ENABLE
  xTaskCreate(
    debugTask,           // Function name
    "DEBUG",             // Text name
    DEBUG_TASK_STACK_SIZE,      // Stack size (bytes)
    NULL,                // Parameter for task
    DEBUG_TASK_PRIORITY, // Priority
    &debugTaskHandle     // Pointer
  );
#endif

#if ENCODER_TASK_ENABLE
  xTaskCreate(
    encoderTask,
    "ENCODER_TASK",
    ENCODER_TASK_STACK_SIZE,
    NULL,
    ENCODER_TASK_PRIORITY,
    &encoderTaskHandle
  );
#endif

#if SERIAL_TASK_ENABLE
  xTaskCreate(
    serialTask,
    "SERIAL_TASK",
    SERIAL_TASK_STACK_SIZE,
    NULL,
    SERIAL_TASK_PRIORITY,
    &serialTaskHandle
  );
#endif

#if CONTROL_TASK_ENABLE
  xTaskCreate(
    controlTask,
    "CONTROL_TASK",
    CONTROL_TASK_STACK_SIZE,
    NULL,
    CONTROL_TASK_PRIORITY,
    &controlTaskHandle
  );
#endif

  Serial.println("End setup");
}

// --- LOOP --- //

void loop() {
  // leave empty but do not remove delay
  // i still don't know why but why not if it works, yknow?
  vTaskDelay(pdMS_TO_TICKS(1000));
}
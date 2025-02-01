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
    debugStackSize,      // Stack size (bytes)
    NULL,                // Parameter for task
    DEBUG_TASK_PRIORITY, // Priority
    &debugTaskHandle     // Pointer
  );
#endif

#if ARM_TASK_ENABLE
  xTaskCreate(
    armTask,
    "ARM_TASK",
    armStackSize,
    NULL,
    ARM_TASK_PRIORITY,
    &armTaskHandle
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
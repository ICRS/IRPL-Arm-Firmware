// === INCLUDE === //

#include "tasks.h"
#include "config.h"

#include "freertos/task.h"
#include <Wire.h>

// === GLOBAL VARIABLES === //

// Global variables
std::array<float, N_ENCODERS> desiredAngleArray = {0, 50, 50, 20};
std::array<float, N_ENCODERS> currentAngleArray= {0, 50, 50, 20};

// Hardware timers
hw_timer_t *encoderTimer = NULL;

// === SETUP === //

void setup() {

  Serial.begin(115200);
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
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

  encoderTimer = timerBegin(1, HARDWARE_TIMER_PRESCALER, true);
  timerAttachInterrupt(encoderTimer, sampleEncodersISR, true);
  timerAlarmWrite(encoderTimer, (APB_CLK_FREQ/HARDWARE_TIMER_PRESCALER)/ENCODER_TASK_FREQUENCY, true);
  timerAlarmEnable(encoderTimer);
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